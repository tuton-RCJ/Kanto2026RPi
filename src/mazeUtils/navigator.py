"""
@brief 移動ロジック、被災者救助シーケンス、自己位置推定の補正を行うモジュール
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import numpy as np

from . import mazeConstraints, mazeEnums
from .device import LiDAR, buzzerSongs, deviceConstraints, deviceEnums

from .action import Action
from .perception import Perception
from .robot import Robot


def _regulationAngle(angle: int | float) -> int | float:
    """
    @brief 角度を-180〜180度の範囲に正規化する
    @param angle 入力角度
    @return 正規化された角度
    """
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle


def _getTurnDirection(fromDir: int, toDir: int) -> mazeEnums.turnDirection:
    """
    @brief 回転方向を決定する
    @param fromDir 現在の角度
    @param toDir 目標の角度
    @return 回転方向
    """
    turnAngle = fromDir - toDir
    if turnAngle > 180:
        turnAngle -= 360
    if turnAngle < -180:
        turnAngle += 360

    if turnAngle > 0:
        return mazeEnums.turnDirection.RIGHT
    else:
        return mazeEnums.turnDirection.LEFT


def _getClosestDirectionsForCameraHeading(
    cameraHeadingDeg: float,
) -> list[mazeEnums.absDirection]:
    """
    @brief カメラ向きに最も近い方位（複数の場合あり）を取得する
    @param cameraHeadingDeg カメラの向き（度）
    @return 最も近い方位のリスト
    """
    diffs: list[tuple[mazeEnums.absDirection, float]] = []
    for direction in [
        mazeEnums.absDirection.NORTH,
        mazeEnums.absDirection.EAST,
        mazeEnums.absDirection.SOUTH,
        mazeEnums.absDirection.WEST,
    ]:
        diffs.append(
            (direction, abs(_regulationAngle(cameraHeadingDeg - direction.value)))
        )
    diffs.sort(key=lambda x: x[1])
    closestDir, closestDiff = diffs[0]
    secondDir, secondDiff = diffs[1]

    if (secondDiff - closestDiff) <= mazeConstraints.VICTIM_TURN_DOUBLE_ADD_DIFF_DEG:
        return [closestDir, secondDir]
    return [closestDir]


def _getSteerSpeed(
    leftWallDist: float,
    rightWallDist: float,
    targetDistDiff: float,
    turnAngle: float,
) -> dict[deviceEnums.Side, int]:
    """
    @brief 壁追従とジャイロ補正を組み合わせたステアリング速度を計算する
    @param leftWallDist 左壁までの距離
    @param rightWallDist 右壁までの距離
    @param targetDistDiff 目標距離差係数
    @param turnAngle 現在の角度誤差
    @return 各サイドのモータ速度
    """
    gyroSteer = turnAngle * mazeConstraints.STRAGIHT_GYRO_P_GAIN
    wallSteer = 0.0

    enableDist = mazeConstraints.WALL_FOLLOW_ENABLE_DIST_CM
    targetDist = mazeConstraints.WALL_FOLLOW_TARGET_DIST_CM

    if (leftWallDist <= enableDist or rightWallDist <= enableDist) and abs(turnAngle) <= mazeConstraints.WALL_FOLLOW_GYRO_ERR_MAX_DEG:
        if leftWallDist <= enableDist and rightWallDist <= enableDist:
            wallError = rightWallDist - leftWallDist
        elif leftWallDist <= enableDist:
            wallError = targetDist - leftWallDist
        else:
            wallError = rightWallDist - targetDist

        wallSteer = wallError * mazeConstraints.WALL_FOLLOW_P_GAIN
        wallSteer = max(min(wallSteer, mazeConstraints.WALL_FOLLOW_MAX_STEER), -mazeConstraints.WALL_FOLLOW_MAX_STEER)

    steer = gyroSteer + wallSteer

    baseLeft = (mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT] * targetDistDiff + 20)
    baseRight = (mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT] * targetDistDiff + 20)

    leftSpeed = int(max(min(100, baseLeft + steer), -100))
    rightSpeed = int(max(min(100, baseRight - steer), -100))

    return {deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed}


class Navigator:
    """
    @brief 移動ロジック、被災者救助、自己位置推定補正を管理するクラス
    """

    def __init__(
        self,
        robotInstance: "Robot",
        perceptionInstance: "Perception",
        actionInstance: "Action",
    ) -> None:
        """
        @brief Navigatorクラスの初期化
        @param robotInstance Robotインスタンス
        @param perceptionInstance Perceptionインスタンス
        @param actionInstance Actionインスタンス
        """
        self._robot = robotInstance
        self._perception = perceptionInstance
        self._action = actionInstance

    def _debugPrint(self, *message: object) -> None:
        """
        @brief デバッグ出力
        @param message 出力メッセージ
        """
        if mazeConstraints.DEBUG_MODE:
            print(*message)

    def _checkLackOfProgress(self) -> bool:
        """
        @brief Lack of Progress状態をチェックする
        @return トグルスイッチが押されていたらTrue
        """
        self._robot.update()
        if self._perception.isSwitchPressed():
            self._action.stop()
            return True
        return False

    def moveTile(self, direction: mazeEnums.absDirection) -> tuple[bool, bool]:
        """
        @brief 指定方向へ1マス移動する
        @param direction 移動方向
        @return (黒タイル検出フラグ, LoP発生フラグ)のタプル
        """
        self._robot.update()
        if self._perception.isSwitchPressed():
            self._action.stop()
            return False, True

        lidarWorker = self._robot.getLidarWorker()
        lidarWorker.waitFirst(timeoutSec=1.0)

        mapInstance = self._robot.mapInstance
        print(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")

        self.turnToDirection(direction.value, searchVictim=True)
        self._action.stop()

        isBlack = self._goStraight()
        if isBlack or self._perception.isSwitchPressed():
            return isBlack, self._perception.isSwitchPressed()

        latestPoints, _ = lidarWorker.getLatest()
        self._detectWall(latestPoints)
        tileType = self._perception.detectTileColor()

        if self._perception.isSilverTile():
            self._action.playMusic(buzzerSongs.checkpoint)
            mapInstance.setTileType(mazeEnums.tileType.SILVER)
            tileType = mazeEnums.tileType.SILVER
        else:
            mapInstance.setTileType(tileType if tileType != mazeEnums.tileType.BLACK else mazeEnums.tileType.EMPTY)

        print(f"decided tile type: {tileType}")

        if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
            self._action.playMusic(buzzerSongs.swamp)
            time.sleep(5.2)

        if mapInstance.getTileType() != mazeEnums.tileType.EMPTY:
            print(f"Moved to {mapInstance.currentPosition}, Tile type: {mapInstance.getTileType()}, Wall types: {mapInstance.getWallType()}")

        self._action.stop()

        if self._checkLackOfProgress():
            return False, True

        return False, False

    def turnToDirection(
        self, targetDir: int, searchVictim: bool = False
    ) -> None:
        """
        @brief 指定角度への回転制御と、引数に応じて被災者検知を行う
        @param targetDir 目標の絶対方向
        @param searchVictim 回転中に被災者を探すかどうか
        """

        currentHeading = self._perception.getHeading()

        self._debugPrint(f"current heading: {currentHeading}, target dir: {targetDir}")

        while (abs(_regulationAngle(self._perception.getHeading() - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG):
            if self._checkLackOfProgress():
                return

            if _getTurnDirection(self._perception.getHeading(), targetDir) == mazeEnums.turnDirection.RIGHT:
                self._action.turnRight(50)
            else:
                self._action.turnLeft(50)

            if searchVictim:
                self._scanAndRescueVictimDuringTurn()
                
            self._robot.update()

        self._action.stop()

        self._debugPrint(f"stopped turning at heading: {self._perception.getHeading()}, diff: {abs(_regulationAngle(self._perception.getHeading() - targetDir))}")

        while (abs(_regulationAngle(self._perception.getHeading() - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX):
            if (_getTurnDirection(self._perception.getHeading(), targetDir) == mazeEnums.turnDirection.RIGHT):
                self._action.turnRight(5)
            else:
                self._action.turnLeft(5)

            if self._checkLackOfProgress():
                return

        self._action.stop()

        self._debugPrint(f"Turned to heading: {self._perception.getHeading()} deg")

    def _goStraight(self) -> bool:
        """
        @brief 直進制御を行う
        @return 黒タイル検出フラグ
        """
        mapInstance = self._robot.mapInstance
        lidarWorker = self._robot.getLidarWorker()

        self._robot.update()

        if self._checkLackOfProgress():
            return False

        points, _ = lidarWorker.getLatest()
        if points is None:
            points = lidarWorker.waitFirst(timeoutSec=1.0)

        nearestLiDARAngle = (0 if LiDAR.getCertainAngleDist(0, points) < LiDAR.getCertainAngleDist(180, points) else 180)
        oldDist = LiDAR.getCertainAngleDist(nearestLiDARAngle - self._perception.getHeading() + mapInstance.frontDirection.value, points)

        self._action.setMotor(mazeConstraints.GO_STRAIGHT_MAX_SPEED)

        littleFowardFlag = False
        isRamp = False

        startTime = time.time()
        practicalMoveTime = 0.0
        oldTime = startTime

        cameraBlackTileDetected = False
        isWallAhead = {s: self._perception.isWallAheadTile(s, points) for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
        avoidVictim: dict[deviceEnums.Side, set] = {s: set() for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}

        for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            aheadVictims = mapInstance.getVictimTypes(mapInstance.frontDirection)[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)]
            for v in aheadVictims:
                avoidVictim[s].add(v)

            currentVictims = mapInstance.getVictimTypes()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)]
            for v in currentVictims:
                avoidVictim[s].add(v)

        consequentSearchRes: dict[deviceEnums.Side, deviceEnums.UnitVStatus | None] = {s: None for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
        beforeDist = oldDist

        while True:
            if not cameraBlackTileDetected:
                isBlackTileByCam = self._perception.detectTileColorByCamera()
                cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

            self._robot.update()
            if self._checkLackOfProgress():
                return False

            scanPoints, _ = lidarWorker.getLatest()
            if scanPoints is None:
                continue

            currentHeading = self._perception.getHeading()
            currentDist = LiDAR.getCertainAngleDist(nearestLiDARAngle - currentHeading + mapInstance.frontDirection.value, scanPoints)
            turnAngle = _regulationAngle(currentHeading - mapInstance.frontDirection.value)
            leftWallDist = LiDAR.getCertainAngleDist(90 - currentHeading + mapInstance.frontDirection.value, scanPoints)
            rightWallDist = LiDAR.getCertainAngleDist(270 - currentHeading + mapInstance.frontDirection.value, scanPoints)
            tofDist = self._perception.getTofDistance()
            targetDistDiff = math.sqrt(max(min(1 - abs(oldDist - currentDist) / 30, (tofDist[0] - 15) / 15), 0))

            rollValue = self._perception.getRoll()
            minRoll = min(rollValue, 360 - rollValue)
            maxLidarDist = max(LiDAR.getCertainAngleDist([0, 90, 180, 270], scanPoints))

            if not isRamp and ((90 > minRoll > mazeConstraints.RAMP_DEG_THRESHOLD) or maxLidarDist > 250 or abs(currentDist - beforeDist) > mazeConstraints.MIN_THERESHOULD_FOR_DIFF):
                isRamp = True
                print(f"Ramp detected! roll: {rollValue} deg")

            if not isRamp:
                steerSpeeds = _getSteerSpeed(leftWallDist, rightWallDist, targetDistDiff, turnAngle)
                self._action.setMotor(steerSpeeds)
            else:
                self._action.setMotor(mazeConstraints.GO_STRAIGHT_MAX_SPEED)

            if (self._perception.detectTileColor() == mazeEnums.tileType.BLACK and cameraBlackTileDetected):
                self._action.stop()
                mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=mapInstance.frontDirection)
                print("Black tile detected! Stopping movement. Starting escape maneuver.")
                self._escapeBlackTile(nearestLiDARAngle, oldDist)
                return True

            frontDist = LiDAR.getCertainAngleDist(-currentHeading + mapInstance.frontDirection.value, scanPoints)
            moveCondition = (abs(oldDist - currentDist) > mazeConstraints.MOVE_THRESHOLD_CM or frontDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM)
            rampCondition = not (90 > minRoll > mazeConstraints.RAMP_DEG_THRESHOLD)

            if moveCondition and rampCondition and not isRamp:
                self._action.stop()
                if (mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM < currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM + 15):
                    littleFowardFlag = True
                break

            escapeFlag = False
            timeBeforeEscape = time.time()
            pressedSide = self._perception.isObstaclePressed()
            if pressedSide is not None:
                self._action.escapeObstacle(pressedSide)
                self._action.setMotor(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
                escapeFlag = True
                practicalMoveTime -= time.time() - timeBeforeEscape

            if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC and isRamp:
                self._action.stop()
                break

            isRedTile = self._perception.detectTileColor() == mazeEnums.tileType.RED

            for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                if isRamp:
                    continue

                practicalMoveTime = self._checkAndRescueVictimDuringStraight(side, isWallAhead, avoidVictim, consequentSearchRes, isRedTile, oldDist, currentDist, practicalMoveTime)

                self._debugPrint(f"isramp: {isRamp}, roll: {rollValue} deg, practicalMoveTime: {practicalMoveTime} sec, currentDist: {currentDist} cm, oldDist: {oldDist} cm")

            rollCos = np.cos(np.radians(abs(rollValue)))
            rollMultiplier = 1 if rollValue > 180 else 0.9
            deltaTime = ((time.time() - oldTime) if not escapeFlag else (timeBeforeEscape - oldTime))
            practicalMoveTime += deltaTime * rollCos * rollMultiplier
            oldTime = time.time()
            beforeDist = currentDist

        if littleFowardFlag:
            self._action.setMotor(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
            self._debugPrint("Little forward to adjust position")

            while True:
                if self._checkLackOfProgress():
                    return False

                currentDist = self._perception.getTofDistance()[0]
                if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                    break

        self._action.stop()
        mapInstance.moveTo(mapInstance.frontDirection)
        return False

    def _scanAndRescueVictimDuringTurn(self) -> None:
        """
        @brief 回転中に被災者を検知し、レスキューキットを排出する
        """
        mapInstance = self._robot.mapInstance
        victimInfo = self._perception.getVictimInfo()

        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            if victimInfo[side] == deviceEnums.UnitVStatus.NOTHING:
                continue

            cameraHeadingDeg = (self._perception.getHeading() + (90 if side == deviceEnums.Side.LEFT else 270)) % 360
            targetDirections = _getClosestDirectionsForCameraHeading(cameraHeadingDeg)

            currentVictimTypes = mapInstance.getVictimTypes()
            alreadyRecorded = any(victimInfo[side] in currentVictimTypes[d] for d in targetDirections)

            if not alreadyRecorded:
                print(f"Find victim on {side} side during turn: {victimInfo[side]} -> directions: {[d.name for d in targetDirections]}")

                self._action.stop()
                for d in targetDirections:
                    mapInstance.addVictimType(victimInfo[side], d)
                self._dropRescueKit(victimInfo, side)
                print(f"Dropped rescue kit, detected victim info: {victimInfo}")

    def _scanAndRescueVictim(self) -> None:
        """
        @brief 現在位置で被災者を検知し、レスキューキットを排出する
        """
        mapInstance = self._robot.mapInstance
        leftFlag = True
        rightFlag = True
        t = time.time()

        while time.time() - t < 0.5:
            if self._perception.isSwitchPressed():
                self._action.stop()
                return

            victimInfo = self._perception.getVictimInfo()
            tileType = self._perception.detectTileColor()

            for side, flag in [
                (deviceEnums.Side.LEFT, leftFlag),
                (deviceEnums.Side.RIGHT, rightFlag),
            ]:
                if not flag:
                    continue

                dirAngle = (mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360
                direction = mazeEnums.absDirection(dirAngle)

                if (victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and victimInfo[side] not in mapInstance.getVictimTypes()[direction] and mapInstance.getWallType()[direction] == mazeEnums.wallType.WALL):
                    if not (tileType == mazeEnums.tileType.RED and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM):
                        print(f"Find victim on {side} side: {victimInfo[side]}")
                        mapInstance.addVictimType(victimInfo[side], direction)
                        self._dropRescueKit(victimInfo, side)
                        print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                        if side == deviceEnums.Side.LEFT:
                            leftFlag = False
                        else:
                            rightFlag = False

            self._robot.update()

    def _checkAndRescueVictimDuringStraight(
        self,
        side: deviceEnums.Side,
        isWallAhead: dict[deviceEnums.Side, bool],
        avoidVictim: dict[deviceEnums.Side, set],
        consequentSearchRes: dict[deviceEnums.Side, deviceEnums.UnitVStatus | None],
        isRedTile: bool,
        oldDist: float,
        currentDist: float,
        practicalMoveTime: float,
    ) -> float:
        """
        @brief 直進中に被災者を検知して救助を行う
        @param side 見るサイド
        @param isWallAhead 前方壁の有無
        @param avoidVictim 回避する被災者の種類
        @param consequentSearchRes 直進中に検知した被災者の情報
        @param isRedTile 赤タイル上かどうか
        @param oldDist 初期距離
        @param currentDist 現在距離
        @param practicalMoveTime 実効移動時間
        @return 更新された実効移動時間
        """
        mapInstance = self._robot.mapInstance

        if isWallAhead[side]:
            victimInfo = self._perception.getVictimInfo()
            if (
                victimInfo[side] != deviceEnums.UnitVStatus.NOTHING
                and victimInfo[side] not in avoidVictim[side]
                and consequentSearchRes[side] is None
            ):
                if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                    self._debugPrint("Skipping red victim on red tile")
                    return practicalMoveTime

                self._action.stop()
                t = time.time()
                consequentSearchRes[side] = victimInfo[side]
                print(f"Detected victim info ahead: {victimInfo}")

                dirAngle = (mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360
                direction = mazeEnums.absDirection(dirAngle)

                mapInstance.addVictimType(victimInfo[side], direction)
                mapInstance.addVictimType(victimInfo[side], direction, mapInstance.frontDirection)
                self._dropRescueKit(victimInfo, side)
                practicalMoveTime -= time.time() - t
        else:
            lastUpdateTime = self._perception.getUnitVLastUpdateTime()[side]

            if (30 - abs(oldDist - currentDist) < mazeConstraints.MOVE_THRESHOLD_CM * mazeConstraints.THRESHOLD_SEE_CAM):
                victimInfo = self._perception.getVictimInfo()

                if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                    self._debugPrint("Skipping red victim on red tile")
                    return practicalMoveTime

                dirAngle = (mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360
                direction = mazeEnums.absDirection(dirAngle)
                tofIdx = 1 if side == deviceEnums.Side.LEFT else 3

                if (victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and victimInfo[side] not in mapInstance.getVictimTypes(mapInstance.frontDirection)[direction] and self._perception.getTofDistance()[tofIdx] < deviceConstraints.WALL_DETECTION_THRESHOLD_CM):
                    print(f"Detected victim info during movement: {victimInfo}")

                    self._action.stop()
                    t = time.time()

                    mapInstance.addVictimType(victimInfo[side], direction, mapInstance.frontDirection)
                    self._dropRescueKit(victimInfo, side)
                    practicalMoveTime -= time.time() - t

            if (abs(oldDist - currentDist) - lastUpdateTime / 1000 * 20 < mazeConstraints.MOVE_THRESHOLD_CM * mazeConstraints.THRESHOLD_SEE_CAM):
                victimInfo = self._perception.getVictimInfo()

                dirAngle = (
                    mapInstance.frontDirection.value
                    + (90 if side == deviceEnums.Side.LEFT else 270)
                ) % 360
                direction = mazeEnums.absDirection(dirAngle)

                if (victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and victimInfo[side] not in mapInstance.getVictimTypes()[direction] and mapInstance.getWallType()[direction] == mazeEnums.wallType.WALL):
                    if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                        self._debugPrint("Skipping red victim on red tile")
                        return practicalMoveTime

                    self._action.stop()
                    t = time.time()

                    print(f"Detected victim info during movement needing rescue kit drop: {victimInfo}")

                    mapInstance.addVictimType(victimInfo[side], direction)
                    self._dropRescueKit(victimInfo, side)
                    practicalMoveTime -= time.time() - t

        return practicalMoveTime

    def _escapeBlackTile(self, nearestLiDARAngle: int, oldDist: float) -> None:
        """
        @brief 黒タイル検知時の緊急停止と後退を行う
        @param nearestLiDARAngle 最も壁と近い LiDAR の角度
        @param oldDist 初期距離
        """
        mapInstance = self._robot.mapInstance
        lidarWorker = self._robot.getLidarWorker()
        sign = -1 if nearestLiDARAngle == 0 else 1
        escapeStartTime = time.time()
        escapeTimeoutSec = 3.0

        while True:
            if time.time() - escapeStartTime > escapeTimeoutSec:
                self._debugPrint("Escape timeout reached, stopping escape maneuver")
                break

            latestPoints, _ = lidarWorker.getLatest()
            if latestPoints is None:
                continue

            currentHeading = self._perception.getHeading()
            currentDist = LiDAR.getCertainAngleDist(
                nearestLiDARAngle - currentHeading + mapInstance.frontDirection.value,
                latestPoints,
            )

            if sign * (currentDist - oldDist) <= 0:
                break

            self._action.setMotor(
                {deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30}
            )

            if self._checkLackOfProgress():
                return

        latestPoints, _ = lidarWorker.getLatest()
        if latestPoints is not None:
            currentHeading = self._perception.getHeading()
            self._debugPrint(
                f"Escape maneuver complete. Current Distance: {LiDAR.getCertainAngleDist(nearestLiDARAngle - currentHeading + mapInstance.frontDirection.value, latestPoints)} cm"
            )

    def _dropRescueKit(
        self,
        victimInfo: dict[deviceEnums.Side, deviceEnums.UnitVStatus],
        side: deviceEnums.Side,
    ) -> None:
        """
        @brief 被災者に応じた数の救助キットを投下する
        @param victimInfo 被災者情報
        @param side 投下するサイド
        """
        mapInstance = self._robot.mapInstance
        needRescueKitCount = (victimInfo[side].value - 1) % 3

        colorList = [(0, 255, 0), (255, 255, 0), (255, 0, 0)]
        self._action.flashLED(5, 0.5, color=list(colorList[needRescueKitCount]))

        oppositeFlag = False
        tileColor = self._perception.detectTileColor()

        if (
            tileColor == mazeEnums.tileType.RED
            and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM
        ):
            return

        for i in range(needRescueKitCount):
            dropSide = side if not oppositeFlag else side.opposite()

            if mapInstance.nowRescueKitCount[dropSide] >= 1:
                self.turnToDirection((mapInstance.frontDirection.value + i * mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360)
                mapInstance.dropRescueKit(dropSide, 1)
                self._action.dropKit(dropSide, 1)

                time.sleep(1)

            elif mapInstance.nowRescueKitCount[dropSide.opposite()] >= 1:
                self.turnToDirection((mapInstance.frontDirection.value + 180 + i * mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360)
                mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
                altSide = side.opposite() if not oppositeFlag else side
                mapInstance.dropRescueKit(altSide, 1)
                self._action.dropKit(altSide, 1)
                oppositeFlag = not oppositeFlag

                time.sleep(1)
                
            else:
                self._debugPrint(f"Not enough rescue kits to drop on {side} side.")

        self.turnToDirection(mapInstance.frontDirection.value)

    def _detectWall(self, points: list[LiDAR.Point] | None = None) -> None:
        """
        @brief LiDAR で壁を検出しマップに登録する
        @param points LiDAR の点群データ
        """
        mapInstance = self._robot.mapInstance

        if points is None:
            points = self._perception.getLidarPoints()
        if points is None:
            return

        currentDirVal = mapInstance.frontDirection.value

        for direction in [
            mazeEnums.absDirection.NORTH,
            mazeEnums.absDirection.EAST,
            mazeEnums.absDirection.SOUTH,
            mazeEnums.absDirection.WEST,
        ]:
            angle = (direction.value - currentDirVal + 360) % 360
            dist = LiDAR.getCertainAngleDist(angle, points)
            print(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")

            if mapInstance.getWallType()[direction] == mazeEnums.wallType.UNKNOWN:
                if dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM:
                    mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
                else:
                    mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

    def recoverFromLoP(self) -> None:
        """
        @brief LoP 状態からの復帰動作を行う
        """
        mapInstance = self._robot.mapInstance

        self._robot.update()
        nowAngle = self._perception.getHeading()
        nowDirection = None
        error = 1e9

        for direction in mazeEnums.absDirection:
            diff = abs(nowAngle - direction.value)
            if diff > 180:
                diff = 360 - diff
            if diff < error:
                error = diff
                nowDirection = direction

        mapInstance.loadCache(nowDirection=nowDirection)
        mapInstance.renderKnownTileAndWall()
        time.sleep(1)

        self._robot.update()
        self._detectWall()
        tileType = self._perception.detectTileColor()
        mapInstance.setTileType(tileType)
        self._scanAndRescueVictim()
        self._action.flashLED(0, 0, color=[0, 0, 0])
