import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap
from .device import LiDAR, deviceConstraints, deviceEnums, stm, camera,buzzerSongs
import time
import numpy as np
import math
import threading
from collections import defaultdict
colorSensor = None
pr = None


_lidarScanWorkers: dict[int, "_asyncLidarScanWorker"] = {}
_lidarScanWorkersLock = threading.Lock()


class _asyncLidarScanWorker:
    def __init__(self, lidar: ydlidar.CYdLidar) -> None:
        self._lidar = lidar
        self._cv = threading.Condition()
        self._latest_points: list[LiDAR.Point] | None = None
        self._seq = 0
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="LiDARScanWorker", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            with self._cv:
                if self._stop:
                    return
            try:
                points = LiDAR.getLiDARScan(self._lidar)
            except Exception as e:
                debugPrint(f"LiDAR scan failed: {e}")
                time.sleep(0.005)
                continue
            with self._cv:
                self._latest_points = points
                self._seq += 1
                self._cv.notify_all()

    def waitFirst(self, timeoutSec: float = 1.0) -> list[LiDAR.Point]:
        end = time.time() + timeoutSec
        with self._cv:
            while self._latest_points is None:
                remaining = end - time.time()
                if remaining <= 0:
                    raise TimeoutError("Timed out waiting for first LiDAR scan")
                self._cv.wait(timeout=remaining)
            return self._latest_points

    def getLatest(self) -> tuple[list[LiDAR.Point] | None, int]:
        with self._cv:
            return self._latest_points, self._seq

    def stop(self) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify_all()


def _getLidarScanWorker(lidar: ydlidar.CYdLidar) -> _asyncLidarScanWorker:
    key = id(lidar)
    with _lidarScanWorkersLock:
        worker = _lidarScanWorkers.get(key)
        if worker is None:
            worker = _asyncLidarScanWorker(lidar)
            _lidarScanWorkers[key] = worker
        return worker

def turnOnLED(stmInstance: stm.STM, color: list[int]) -> None:
    """
    @brief LEDを点灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(*color)

def turnOffLED(stmInstance: stm.STM) -> None:
    """
    @brief LEDを消灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(0,0,0)

def flashLED(stmInstance: stm.STM, loopCount: int, intervalSec: float, color: list[int]) -> None:
    """
    @brief LEDを点滅させる
    @param stmInstance: 通信に使用する STM インスタンス
    @param durationSec: 点滅させる時間 (秒)
    @param intervalSec: 点灯と消灯の間隔 (秒)
    """
    for _ in range(loopCount):
        if stmInstance.switch.getToggleSwitch1(): 
            turnOffLED(stmInstance)
            return
        turnOnLED(stmInstance, color)
        t = time.time()
        while time.time() - t < intervalSec:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                turnOffLED(stmInstance)
                return
        turnOffLED(stmInstance)
        t = time.time()
        while time.time() - t < intervalSec:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                turnOffLED(stmInstance)
                return

def debugPrint(*message: object) -> None:
    if mazeConstraints.DEBUG_MODE:
        print(*message)

def detectTileColor() -> mazeEnums.tileType:
    """
    @brief カラーセンサでタイルの色を検出する
    @return: 検出されたタイルの色
    """
    global colorSensor
    if colorSensor is None:
        from .device import colorsensor

        colorSensor = colorsensor.ColorSensor()
    try:
        if not colorSensor.update():
            return mazeEnums.tileType.UNKNOWN
        r, g, b = colorSensor._colorRGB
        if ((mazeConstraints.BLACKTILE_RGB[1][0] < r < mazeConstraints.BLACKTILE_RGB[0][0]) and
            (mazeConstraints.BLACKTILE_RGB[1][1] < g < mazeConstraints.BLACKTILE_RGB[0][1])and
            (mazeConstraints.BLACKTILE_RGB[1][2] < b < mazeConstraints.BLACKTILE_RGB[0][2])):
            return mazeEnums.tileType.BLACK
        elif ((mazeConstraints.BLUETILE_RGB[1][0] < r < mazeConstraints.BLUETILE_RGB[0][0]) and
              (mazeConstraints.BLUETILE_RGB[1][1] < g < mazeConstraints.BLUETILE_RGB[0][1]) and
              (mazeConstraints.BLUETILE_RGB[1][2] < b < mazeConstraints.BLUETILE_RGB[0][2])):
            return mazeEnums.tileType.BLUE
        elif  ((mazeConstraints.REDTILE_RGB[1][0] < r < mazeConstraints.REDTILE_RGB[0][0]) and
              (mazeConstraints.REDTILE_RGB[1][1] < g < mazeConstraints.REDTILE_RGB[0][1]) and
              (mazeConstraints.REDTILE_RGB[1][2] < b < mazeConstraints.REDTILE_RGB[0][2])):
            return mazeEnums.tileType.RED
        else:
            return mazeEnums.tileType.EMPTY
    except Exception as e:
        debugPrint(f"ColorSensor read failed: {e}")
        return mazeEnums.tileType.UNKNOWN

def escapeFromObstacle(deviceEnumsSide: deviceEnums.Side, stmInstance: stm.STM) -> None:
    """
    @brief Loadcell が押されたときに障害物から脱出する動作を行う
    @param deviceEnumsSide: 押された Loadcell の側
    @param stmInstance: 通信に使用する STM インスタンス
    """
    print(f"Escape from obstacle on {deviceEnumsSide} side")
    stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30})
    time.sleep(0.2)
    if deviceEnumsSide == deviceEnums.Side.LEFT:
        stmInstance.sts3032.turnRight(30)
        time.sleep(0.2)
    else:
        stmInstance.sts3032.turnLeft(30)
        time.sleep(0.2)
    stmInstance.sts3032.stop()

def regulationAngle(angle: int) -> int:
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle

def _getClosestDirectionsForCameraHeading(cameraHeadingDeg: float) -> list[mazeEnums.absDirection]:
    diffs: list[tuple[mazeEnums.absDirection, float]] = []
    for direction in [mazeEnums.absDirection.NORTH, mazeEnums.absDirection.EAST, mazeEnums.absDirection.SOUTH, mazeEnums.absDirection.WEST]:
        diffs.append((direction, abs(regulationAngle(cameraHeadingDeg - direction.value))))
    diffs.sort(key=lambda x: x[1])
    closest_dir, closest_diff = diffs[0]
    second_dir, second_diff = diffs[1]

    if (second_diff - closest_diff) <= mazeConstraints.VICTIM_TURN_DOUBLE_ADD_DIFF_DEG:
        return [closest_dir, second_dir]
    return [closest_dir]

def isSilverTile() -> bool:
    """
    @brief カラーセンサで銀タイルを検出する
    @return: 銀タイルが検出されたかどうか
    """
    global pr
    if pr is None:
        from .device import photoReflector

        pr = photoReflector.PhotoReflector()
    return pr.isReflecting()

def checkLackOfProgress(stmInstance: stm.STM):
    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return True
    return False

def turnToCertainDirection(targetDir: int, stmInstance: stm.STM, mapInstance: mazeMap.mazeMap = None, lidarWorker: _asyncLidarScanWorker = None, seacrchVictim: bool = False) -> None:
    """
    @brief 指定した絶対方向に向きながら、被災者を探す
    @param targetDir: 目標の絶対方向 (0-359)
    @param stmInstance: 通信に使用する STM インスタンス
    @param mapInstance: 迷路のマップインスタンス
    @param lidarWorker: LiDARスキャンワーカーのインスタンス
    @param seacrchVictim: 回転中被災者を探すかどうか
    """

    turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)

    stmInstance.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(50)
    debugPrint("current heading:", stmInstance.gyro.getValue().heading, "target:", targetDir)
    
    while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG:
        if checkLackOfProgress(stmInstance):
            stmInstance.sts3032.stop()
            return
        if seacrchVictim and mapInstance is not None and lidarWorker is not None:
            victimInfo = getVictimInfo(stmInstance)
            for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    cameraHeadingDeg = (stmInstance.gyro.getValue().heading + (90 if side == deviceEnums.Side.LEFT else 270)) % 360
                    targetDirections = _getClosestDirectionsForCameraHeading(cameraHeadingDeg)

                    currentVictimTypes = mapInstance.getVictimTypes()
                    alreadyRecorded = any(victimInfo[side] in currentVictimTypes[d] for d in targetDirections)

                    if not alreadyRecorded:
                        print(f"Find victim on {side} side during turn: {victimInfo[side]} -> directions: {[d.name for d in targetDirections]}")
                        
                        stmInstance.sts3032.stop()
                        for d in targetDirections:
                            mapInstance.addVictimType(victimInfo[side], d)
                        dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                        print(f"Dropped rescue kit, detected victim info: {victimInfo}")
        stmInstance.update()

    assert abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) <= mazeConstraints.TURN_THRESHOLD_DEG, f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
    stmInstance.sts3032.stop()

    debugPrint("stopped turning at heading:", stmInstance.gyro.getValue().heading, "diff:" , abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)))

    while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX:
        stmInstance.sts3032.turnRight(5) if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir) == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(5)
        if checkLackOfProgress(stmInstance):
            stmInstance.sts3032.stop()
            return

    assert abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) <= mazeConstraints.TURN_THRESHOLD_DEG_FIX, f"Gyro fine turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
    stmInstance.sts3032.stop()

    debugPrint(f"Turned to heading: {stmInstance.gyro.getValue().heading} deg")
    
def getTurnDirection(fromDir: int, toDir: int) -> mazeEnums.turnDirection:
    turnAngle = fromDir - toDir
    if turnAngle > 180:
        turnAngle -= 360
    if turnAngle < -180:
        turnAngle += 360
    
    if turnAngle > 0:
        return mazeEnums.turnDirection.RIGHT
    else:
        return mazeEnums.turnDirection.LEFT
    
def detectWall(lidar: ydlidar.CYdLidar, mapInstance: mazeMap.mazeMap, points: list[LiDAR.Point] | None = None) -> None:
    if points is None:
        points = LiDAR.getLiDARScan(lidar)
    currentDirVal = mapInstance.frontDirection.value
    for direction in [mazeEnums.absDirection.NORTH, mazeEnums.absDirection.EAST, mazeEnums.absDirection.SOUTH, mazeEnums.absDirection.WEST]:
        angle = (direction.value - currentDirVal + 360) % 360
        dist = LiDAR.getCertainAngleDist(angle, points)
        print(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")
        if mapInstance.getWallType()[direction] == mazeEnums.wallType.UNKNOWN:
            if dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM:
                mapInstance.setWallType(direction, mazeEnums.wallType.WALL)

            else: 
                mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

def getVictimInfo(stmInstance: stm.STM) -> dict[deviceEnums.Side,deviceEnums.UnitVStatus]:
    """ 
    @brief unitv から被災者の情報を取得する
    @param stmInstance: 通信に使用する STM インスタンス
    @return: 各サイドの被災者の種類を示す辞書
    """
    stmInstance.update()
    victimData = stmInstance.unitv.getStatus()
    return victimData

def dropRescueKit(stmInstance: stm.STM, mapInstance: mazeMap.mazeMap, victimInfo: dict[deviceEnums.Side,deviceEnums.UnitVStatus], side: deviceEnums.Side) -> None:
    """
    @brief 指定した側に救助キットを投下する
    @param stmInstance: 通信に使用する STM インスタンス
    @param side: 救助キットを投下する側
    """
    needRescueKitCount = (victimInfo[side].value - 1)%3 
    flashLED(stmInstance, 5, 0.5,color=[(0,255,0),(255,255,0),(255,0,0)][needRescueKitCount])
    oppositeFlag = False
    tileColor = detectTileColor()
    if tileColor == mazeEnums.tileType.RED and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
        return
    for i in range(needRescueKitCount):
        if mapInstance.nowRescueKitCount[side if not oppositeFlag else side.opposite()] >= 1:
            turnToCertainDirection((mapInstance.frontDirection.value + i*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.dropRescueKit(side if not oppositeFlag else side.opposite(), 1,)
            stmInstance.rescuekitservo.dropRescueKit(1, side if not oppositeFlag else side.opposite())
            oldtime = time.time()
            while time.time() - oldtime < 1:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return
        elif mapInstance.nowRescueKitCount[side.opposite() if not oppositeFlag else side] >= 1:
            turnToCertainDirection((mapInstance.frontDirection.value + 180 + i*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
            mapInstance.dropRescueKit(side.opposite() if not oppositeFlag else side, 1)
            stmInstance.rescuekitservo.dropRescueKit(1, side.opposite() if not oppositeFlag else side)
            oppositeFlag = not oppositeFlag
            oldtime = time.time()
            while time.time() - oldtime < 1:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return
        else:
            debugPrint(f"Not enough rescue kits to drop on {side} side.")
    turnToCertainDirection(mapInstance.frontDirection.value, stmInstance)


def vitimToWallType(victim: deviceEnums.UnitVStatus) -> mazeEnums.wallType:
    if victim == deviceEnums.UnitVStatus.H_VICTIM:
        return mazeEnums.wallType.H_VICTIM
    elif victim == deviceEnums.UnitVStatus.S_VICTIM:
        return mazeEnums.wallType.S_VICTIM
    elif victim == deviceEnums.UnitVStatus.U_VICTIM:
        return mazeEnums.wallType.U_VICTIM
    elif victim == deviceEnums.UnitVStatus.Y_VICTIM:
        return mazeEnums.wallType.Y_VICTIM
    elif victim == deviceEnums.UnitVStatus.G_VICTIM:
        return mazeEnums.wallType.G_VICTIM
    elif victim == deviceEnums.UnitVStatus.R_VICTIM:
        return mazeEnums.wallType.R_VICTIM
    else:
        return mazeEnums.wallType.WALL


def escapeFromBlackTile(stmInstance: stm.STM, mapInstance: mazeMap.mazeMap, lidarWorker: _asyncLidarScanWorker, nearestLiDARAngle: int, oldDist: float) -> None:
    sign = -1 if nearestLiDARAngle == 0 else 1
    while True:
        latestPoints, _ = lidarWorker.getLatest()
        if latestPoints is None:
            continue
        if sign * (LiDAR.getCertainAngleDist(nearestLiDARAngle - stmInstance.gyro.getValue().heading + mapInstance.frontDirection.value, latestPoints) - oldDist) <= 0:
            break
        stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -30 , deviceEnums.Side.RIGHT: -30})
        if checkLackOfProgress(stmInstance):
            stmInstance.sts3032.stop()
            return
    latestPoints, _ = lidarWorker.getLatest()
    if latestPoints is not None:
        debugPrint(f"Escape maneuver complete. Current Distance: {LiDAR.getCertainAngleDist(nearestLiDARAngle, latestPoints)} cm")

def getSteerSpeed(leftWallDist: float, rightWallDist: float, targetDistDiff: float, turnAngle: float) -> dict[deviceEnums.Side, int]:
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

    baseLeft = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT] * targetDistDiff + 20
    baseRight = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT] * targetDistDiff + 20

    leftSpeed = int(max(min(100, baseLeft + steer), -100))
    rightSpeed = int(max(min(100, baseRight - steer), -100))

    return {deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed}

def goStraight(mapInstance: mazeMap.mazeMap, stmInstance: stm.STM, lidarWorker:  _asyncLidarScanWorker) -> bool:

    stmInstance.update()
    
    if checkLackOfProgress(stmInstance):
        stmInstance.sts3032.stop()
        return False
    
    points, _ = lidarWorker.getLatest()
    if points is None:
        points = lidarWorker.waitFirst(timeoutSec=1.0)

    nearestLiDARAngle = 0 if LiDAR.getCertainAngleDist(0, points) < LiDAR.getCertainAngleDist(180, points) else 180
    oldDist = LiDAR.getCertainAngleDist(nearestLiDARAngle-stmInstance.gyro.getValue().heading+mapInstance.frontDirection.value, points)

    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    
    littleFowardFlag = False
    isRamp = False
    
    startTime = time.time()
    practicalMoveTime = 0.0
    oldTime = startTime
    
    cameraBlackTileDetected = False
    isWallAhead = {s: LiDAR.isWallAheadTile(points, s) for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    avoidVictim = {s: set() for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    
    for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        aheadVictims = mapInstance.getVictimTypes(mapInstance.frontDirection)[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)]
        for v in aheadVictims:
            avoidVictim[s].add(v)

        currentVictims = mapInstance.getVictimTypes()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)]
        for v in currentVictims:
            avoidVictim[s].add(v)

    consequentSearchRes = {s: None for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    beforeDist = oldDist

    while True:
        if not cameraBlackTileDetected:
            isBlackTileByCam = camera.detectTileColor() == "BLACK"
            cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

        stmInstance.update()
        if checkLackOfProgress(stmInstance):
            stmInstance.sts3032.stop()
            return False
        
        scanPoints, _ = lidarWorker.getLatest()
        if scanPoints is None:
            continue
            
        currentDist = LiDAR.getCertainAngleDist(nearestLiDARAngle-stmInstance.gyro.getValue().heading+mapInstance.frontDirection.value, scanPoints)
        turnAngle = regulationAngle(stmInstance.gyro.getValue().heading - mapInstance.frontDirection.value)
        leftWallDist = LiDAR.getCertainAngleDist(90-stmInstance.gyro.getValue().heading+mapInstance.frontDirection.value, scanPoints)
        rightWallDist = LiDAR.getCertainAngleDist(270-stmInstance.gyro.getValue().heading+mapInstance.frontDirection.value, scanPoints)
        targetDistDiff = math.sqrt(max(min(1-abs(oldDist - currentDist)/30, (stmInstance.tof.getDistance()[0]-15)/15),0))    

        if not isRamp and ((90 > min(stmInstance.gyro.getValue().roll, 360 - stmInstance.gyro.getValue().roll) > mazeConstraints.RAMP_DEG_THRESHOLD) or max(LiDAR.getCertainAngleDist([0,90,180,270], scanPoints)) > 250 or abs(currentDist - beforeDist) > mazeConstraints.MIN_THERESHOULD_FOR_DIFF):
            isRamp = True
            print(f"Ramp detected! roll: {stmInstance.gyro.getValue().roll} deg")

        if not isRamp:
            steerSpeeds = getSteerSpeed(leftWallDist, rightWallDist, targetDistDiff, turnAngle)
            stmInstance.sts3032.setMotorSpeed(steerSpeeds)
        else:
            stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)

        if detectTileColor() == mazeEnums.tileType.BLACK and cameraBlackTileDetected:
            stmInstance.sts3032.stop()
            mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=mapInstance.frontDirection)
            print("Black tile detected! Stopping movement. Starting escape maneuver.")
            escapeFromBlackTile(stmInstance, mapInstance, lidarWorker, nearestLiDARAngle, oldDist)
            return True
        
        if  (abs((oldDist) - (currentDist))> mazeConstraints.MOVE_THRESHOLD_CM or LiDAR.getCertainAngleDist(-stmInstance.gyro.getValue().heading + mapInstance.frontDirection.value, scanPoints) < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM) and (not 90 > min(stmInstance.gyro.getValue().roll, 360 - stmInstance.gyro.getValue().roll) > mazeConstraints.RAMP_DEG_THRESHOLD) and (not isRamp):
            stmInstance.sts3032.stop()
            if mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM < currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM + 15:
                littleFowardFlag = True
            break

        escapeFlag = False
        timeBeforeEscape = time.time()
        if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT] or stmInstance.loadcell.getPressed()[deviceEnums.Side.RIGHT]:
            pressedSide = deviceEnums.Side.LEFT if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT] else deviceEnums.Side.RIGHT
            escapeFromObstacle(pressedSide, stmInstance)
            stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
            escapeFlag = True
            practicalMoveTime -= time.time() - timeBeforeEscape
        
        if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC and isRamp:
            stmInstance.sts3032.stop()
            break
        
        isRedTile = detectTileColor() == mazeEnums.tileType.RED

        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            if isRamp:
                continue
            if isWallAhead[side]:
                victimInfo = stmInstance.unitv.getStatus()
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and (not victimInfo[side] in avoidVictim[side]) and consequentSearchRes[side] is None:
                    if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                        debugPrint("Skipping red victim on red tile")
                        continue
                    stmInstance.sts3032.stop()
                    t = time.time()
                    consequentSearchRes[side] = victimInfo[side]
                    print(f"Detected victim info ahead: {victimInfo}")
                    mapInstance.addVictimType(victimInfo[side], mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360))
                    mapInstance.addVictimType(victimInfo[side], mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360), mapInstance.frontDirection)
                    dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                    practicalMoveTime -= (time.time() - t)
            else:
                lastUpdateTime = stmInstance.unitv.getLastUpdateTime()[side]

                if (30 - abs(oldDist - currentDist) < mazeConstraints.MOVE_THRESHOLD_CM * mazeConstraints.THRESHOLD_SEE_CAM):
                    victimInfo = stmInstance.unitv.getStatus()
                    
                    if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                        debugPrint("Skipping red victim on red tile")
                        continue

                    if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and not victimInfo[side] in mapInstance.getVictimTypes(mapInstance.frontDirection)[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] and stmInstance.tof.getDistance()[1 if side == deviceEnums.Side.LEFT else 3] < deviceConstraints.WALL_DETECTION_THRESHOLD_CM:
                        print(f"Detected victim info during movement: {victimInfo}")
                    
                    stmInstance.sts3032.stop()
                    t = time.time()

                    mapInstance.addVictimType(victimInfo[side], mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360), mapInstance.frontDirection)
                    dropRescueKit(stmInstance, mapInstance, victimInfo, side)

                    practicalMoveTime -= (time.time() - t)


                if (abs(oldDist - currentDist) - lastUpdateTime/1000 * 20) < mazeConstraints.MOVE_THRESHOLD_CM * mazeConstraints.THRESHOLD_SEE_CAM:
                    victimInfo = stmInstance.unitv.getStatus()

                    if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and not victimInfo[side] in mapInstance.getVictimTypes()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] == mazeEnums.wallType.WALL:
                        if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                            debugPrint("Skipping red victim on red tile")
                            continue

                        stmInstance.sts3032.stop()
                        t = time.time()
                        
                        print(f"Detected victim info during movement needing rescue kit drop: {victimInfo}")
                        
                        mapInstance.addVictimType(victimInfo[side], mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360))
                        dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                        practicalMoveTime -= (time.time() - t)

        debugPrint(f"isramp: {isRamp}, roll: {stmInstance.gyro.getValue().roll} deg, practicalMoveTime: {practicalMoveTime} sec, currentDist: {currentDist} cm, oldDist: {oldDist} cm")
        
        practicalMoveTime += ((time.time() - oldTime) if not escapeFlag else (timeBeforeEscape - oldTime))*np.cos(np.radians(abs(stmInstance.gyro.getValue().roll))) * (1 if stmInstance.gyro.getValue().roll > 180 else 0.9) 
        oldTime = time.time()
        beforeDist = currentDist

    if littleFowardFlag:
        stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")

        while True:
            if checkLackOfProgress(stmInstance):
                stmInstance.sts3032.stop()
                return False
            
            currentDist = stmInstance.tof.getDistance()[0]
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
                
    stmInstance.sts3032.stop()   
    mapInstance.moveTo(mapInstance.frontDirection)    
    return False

def rescueVictim(mapInstance: mazeMap.mazeMap, stmInstance: stm.STM) -> None:
    """
    @brief 被災者を救助する動作を行う
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    """     
    stmInstance.update()
    leftFlag = True
    rightFlag = True
    t = time.time()
    while time.time() - t < 0.5:
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return
        victimInfo = getVictimInfo(stmInstance)
        tiletype = detectTileColor()
        if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and not victimInfo[deviceEnums.Side.LEFT] in mapInstance.getVictimTypes()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL and leftFlag:
            if not (tiletype == mazeEnums.tileType.RED and victimInfo[deviceEnums.Side.LEFT] == deviceEnums.UnitVStatus.R_VICTIM):                    
                print(f"Find victim on LEFT side: {victimInfo[deviceEnums.Side.LEFT]}")
                dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.LEFT)
                print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                leftFlag = False

        if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and not victimInfo[deviceEnums.Side.RIGHT] in mapInstance.getVictimTypes()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL and rightFlag:
            if not (tiletype == mazeEnums.tileType.RED and victimInfo[deviceEnums.Side.RIGHT] == deviceEnums.UnitVStatus.R_VICTIM):
                print(f"Find victim on RIGHT side: {victimInfo[deviceEnums.Side.RIGHT]}")  
                dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
                print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                rightFlag = False
        stmInstance.update()

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stmInstance: stm.STM, lidar: ydlidar.CYdLidar) -> tuple[bool,bool]:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True

    lidarWorker = _getLidarScanWorker(lidar)
    # Ensure we have at least one scan before starting motion logic.
    lidarWorker.waitFirst(timeoutSec=1.0)

    print(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")
    turnToCertainDirection(direction.value, stmInstance, mapInstance, lidarWorker, seacrchVictim=True)
    stmInstance.sts3032.stop()
    
    isBlack = goStraight(mapInstance, stmInstance, lidarWorker)
    if isBlack or stmInstance.switch.getToggleSwitch1():
        return isBlack, stmInstance.switch.getToggleSwitch1()
    
    latestPoints, _ = lidarWorker.getLatest()
    detectWall(lidar, mapInstance, points=latestPoints)
    tileType = detectTileColor()

    if isSilverTile():
        stmInstance.buzzer.playMusic(buzzerSongs.checkpoint)
        mapInstance.setTileType(mazeEnums.tileType.SILVER)
        tileType = mazeEnums.tileType.SILVER
    else:
        mapInstance.setTileType(tileType if tileType != mazeEnums.tileType.BLACK else mazeEnums.tileType.EMPTY)

    print(f"decided tile type: {tileType}")

    if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
        stmInstance.buzzer.playMusic(buzzerSongs.swamp)
        time.sleep(5.2)
    if mapInstance.getTileType() != mazeEnums.tileType.EMPTY:
        print(f"Moved to {mapInstance.currentPosition}, Tile type: {mapInstance.getTileType()}, Wall types: {mapInstance.getWallType()}")

    stmInstance.sts3032.stop()

    if checkLackOfProgress(stmInstance):
        return False, True
    
    return False, False

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stmInstance: stm.STM, lidar: ydlidar.CYdLidar) -> tuple[bool,bool]:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """
    isBlack, stopped = moveTile(direction, mapInstance, stmInstance, lidar)

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return isBlack, True
    return isBlack, stopped

