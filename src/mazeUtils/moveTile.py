import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap
from .device import LiDAR, deviceConstraints, deviceEnums, stm, camera, buzzerSongs
from config import get_logger
import time
import threading
import numpy as np
import math
from collections import defaultdict

logger = get_logger(__name__)

colorSensor = None
pr = None

lastDist = {
    mazeEnums.absDirection.NORTH: 0,
    mazeEnums.absDirection.EAST: 0,
    mazeEnums.absDirection.SOUTH: 0,
    mazeEnums.absDirection.WEST: 0,
}


def turnOnLED(
    stmInstance: stm.STM, mapInstance: mazeMap.mazeMap, color: tuple[int, int, int]
) -> None:
    """
    @brief LEDを点灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(*color)
    mapInstance.arduinoNanoEvery.victimled(color)


def turnOffLED(stmInstance: stm.STM, mapInstance: mazeMap.mazeMap) -> None:
    """
    @brief LEDを消灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(0, 0, 0)
    mapInstance.arduinoNanoEvery.victimled((0, 0, 0))


def flashLED(
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    loopCount: int,
    intervalSec: float,
    color: tuple[int, int, int],
) -> None:
    """
    @brief LEDを点滅させる
    @param stmInstance: 通信に使用する STM インスタンス
    @param mapInstance: マップインスタンス
    @param durationSec: 点滅させる時間 (秒)
    @param intervalSec: 点灯と消灯の間隔 (秒)
    """
    for _ in range(loopCount):
        if stmInstance.switch.getToggleSwitch1():
            turnOffLED(stmInstance, mapInstance)
            return
        turnOnLED(stmInstance, mapInstance, color)
        t = time.time()
        while time.time() - t < intervalSec:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                turnOffLED(stmInstance, mapInstance)
                return
        turnOffLED(stmInstance, mapInstance)
        t = time.time()
        while time.time() - t < intervalSec:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                turnOffLED(stmInstance, mapInstance)
                return


def debugPrint(*message: object) -> None:
    if mazeConstraints.DEBUG_MODE:
        logger.debug(" ".join(str(m) for m in message))


class LiDARScanCache:
    def __init__(
        self, lidar: ydlidar.CYdLidar, update_interval_sec: float = 0.13
    ) -> None:
        self._lidar = lidar
        self._update_interval_sec = update_interval_sec
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_points = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_latest(self):
        with self._lock:
            return self._latest_points

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                points = LiDAR.getLiDARScan(self._lidar)
                with self._lock:
                    self._latest_points = points
            except Exception as e:
                debugPrint(f"LiDAR scan failed: {e}")
            time.sleep(self._update_interval_sec)


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
        if (
            (
                mazeConstraints.BLACKTILE_RGB[1][0]
                < r
                < mazeConstraints.BLACKTILE_RGB[0][0]
            )
            and (
                mazeConstraints.BLACKTILE_RGB[1][1]
                < g
                < mazeConstraints.BLACKTILE_RGB[0][1]
            )
            and (
                mazeConstraints.BLACKTILE_RGB[1][2]
                < b
                < mazeConstraints.BLACKTILE_RGB[0][2]
            )
        ):
            return mazeEnums.tileType.BLACK
        elif (
            (
                mazeConstraints.BLUETILE_RGB[1][0]
                < r
                < mazeConstraints.BLUETILE_RGB[0][0]
            )
            and (
                mazeConstraints.BLUETILE_RGB[1][1]
                < g
                < mazeConstraints.BLUETILE_RGB[0][1]
            )
            and (
                mazeConstraints.BLUETILE_RGB[1][2]
                < b
                < mazeConstraints.BLUETILE_RGB[0][2]
            )
        ):
            return mazeEnums.tileType.BLUE
        elif (
            (mazeConstraints.REDTILE_RGB[1][0] < r < mazeConstraints.REDTILE_RGB[0][0])
            and (
                mazeConstraints.REDTILE_RGB[1][1]
                < g
                < mazeConstraints.REDTILE_RGB[0][1]
            )
            and (
                mazeConstraints.REDTILE_RGB[1][2]
                < b
                < mazeConstraints.REDTILE_RGB[0][2]
            )
        ):
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
    logger.warning(f"Escape from obstacle on {deviceEnumsSide} side")
    stmInstance.sts3032.setMotorSpeed(
        {deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30}
    )
    time.sleep(0.2)
    if deviceEnumsSide == deviceEnums.Side.LEFT:
        stmInstance.sts3032.turnRight(30)
        time.sleep(0.2)
    else:
        stmInstance.sts3032.turnLeft(30)
        time.sleep(0.2)
    stmInstance.sts3032.stop()


def escapeFromBlackTile(
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    direction: mazeEnums.absDirection,
    practicalMoveTime: float,
    cameraBlackTileDetected: bool,
) -> bool:
    tempTileColor = detectTileColor()
    if tempTileColor == mazeEnums.tileType.BLACK and cameraBlackTileDetected:
        stmInstance.sts3032.stop()
        mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
        logger.info("Black tile detected! Stopping movement. Starting escape maneuver.")
        startEscapeTime = time.time()
        stmInstance.sts3032.setMotorSpeed(
            {deviceEnums.Side.LEFT: -50, deviceEnums.Side.RIGHT: -50}
        )
        while time.time() - startEscapeTime < practicalMoveTime:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
        stmInstance.sts3032.stop()
        debugPrint(f"Escape maneuver complete.")
        return True
    else:
        return False


def regulationAngle(angle: int | float) -> int | float:
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle


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


def getQuantizedDir(dir: int | float) -> list[mazeEnums.absDirection]:
    """
    @brief 方向を最も近い2方向に量子化する
    @param dir: 量子化する方向 (0-359)
    @return: 量子化された方向のタプル
    """
    dir = dir % 360
    directions = [
        mazeEnums.absDirection.NORTH,
        mazeEnums.absDirection.EAST,
        mazeEnums.absDirection.SOUTH,
        mazeEnums.absDirection.WEST,
    ]
    diffs = [abs(regulationAngle(dir - d.value)) for d in directions]
    sortedIndices = np.argsort(diffs)
    return [directions[sortedIndices[0]], directions[sortedIndices[1]]]


def turnToCertainDirection(
    targetDir: int | float,
    stmInstance: stm.STM,
    rescueVictim: bool = False,
    mapInstance: mazeMap.mazeMap | None = None,
) -> None:
    """
    @brief 指定した絶対方向に向く
    @param targetDir: 目標の絶対方向 (0-359)
    @param stmInstance: 通信に使用する STM インスタンス
    """

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return
    if rescueVictim and (mapInstance is None):
        raise ValueError("mapInstance must be provided when rescueVictim is True")

    if (not mazeConstraints.USE_PD_FOR_TURNING) or rescueVictim:
        turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)

        (
            stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD)
            if turnDirection == mazeEnums.turnDirection.RIGHT
            else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD)
        )
        debugPrint(
            "current heading:",
            stmInstance.gyro.getValue().heading,
            "target:",
            targetDir,
        )
        oldTurnDir = turnDirection
        rescueStopped = False
        while (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            > mazeConstraints.TURN_THRESHOLD_DEG
        ):
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            if rescueVictim:
                for s in deviceEnums.Side:
                    victimInfo = getVictimInfo(stmInstance)
                    tiletype = detectTileColor()
                    if (
                        victimInfo[s] != deviceEnums.UnitVStatus.NOTHING
                        and mapInstance.isSeenVictimType(
                            getQuantizedDir(
                                stmInstance.gyro.getValue().heading
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            ),
                            victimInfo[s],
                        )
                        == False
                        and mapInstance.getWallType()[
                            getQuantizedDir(
                                stmInstance.gyro.getValue().heading
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )[0]
                        ]
                        == mazeEnums.wallType.WALL
                        and mapInstance.getWallType()[
                            getQuantizedDir(
                                stmInstance.gyro.getValue().heading
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )[1]
                        ]
                        == mazeEnums.wallType.WALL
                    ):
                        if not (
                            tiletype == mazeEnums.tileType.RED
                            and victimInfo[s] == deviceEnums.UnitVStatus.R_VICTIM
                        ):
                            stmInstance.sts3032.stop()
                            rescueStopped = True
                            logger.info(
                                f"Find victim on {'LEFT' if s == deviceEnums.Side.LEFT else 'RIGHT'} side during turn: {victimInfo[s]}"
                            )
                            mapInstance.addSeenVictimType(
                                getQuantizedDir(
                                    stmInstance.gyro.getValue().heading
                                    + (90 if s == deviceEnums.Side.LEFT else 270)
                                ),
                                victimInfo[s],
                            )
                            dropRescueKit(stmInstance, mapInstance, victimInfo, s)
                            logger.info(
                                f"Dropped rescue kit, detected victim info: {victimInfo}"
                            )
            turnDirection = getTurnDirection(
                stmInstance.gyro.getValue().heading, targetDir
            )
            if rescueStopped or turnDirection != oldTurnDir:
                (
                    stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD)
                    if turnDirection == mazeEnums.turnDirection.RIGHT
                    else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD)
                )
                oldTurnDir = turnDirection
                rescueStopped = False

        assert (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            <= mazeConstraints.TURN_THRESHOLD_DEG
        ), f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
        stmInstance.sts3032.stop()

        logger.debug(
            f"stopped turning at heading: {stmInstance.gyro.getValue().heading}, "
            f"diff: {abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))}"
        )
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return
        oldTurnDir = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
        (
            stmInstance.sts3032.turnRight(5)
            if oldTurnDir == mazeEnums.turnDirection.RIGHT
            else stmInstance.sts3032.turnLeft(5)
        )
        while (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            > mazeConstraints.TURN_THRESHOLD_DEG_FIX
        ):
            if oldTurnDir != getTurnDirection(
                stmInstance.gyro.getValue().heading, targetDir
            ):
                (
                    stmInstance.sts3032.turnRight(5)
                    if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
                    == mazeEnums.turnDirection.RIGHT
                    else stmInstance.sts3032.turnLeft(5)
                )
                oldTurnDir = getTurnDirection(
                    stmInstance.gyro.getValue().heading, targetDir
                )
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
        stmInstance.sts3032.stop()
    else:  # PD制御
        turnAngle = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        debugPrint(
            f"Turning to {targetDir} deg, current heading: {stmInstance.gyro.getValue().heading} deg, turnAngle: {turnAngle} deg"
        )

        oldError = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        oldTime = time.time()
        while (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            > mazeConstraints.TURN_THRESHOLD_DEG_FIX
        ):
            if time.time() - oldTime > mazeConstraints.TIMEOUT_FOR_TURNING_SEC:
                debugPrint("Turn timeout reached.")
                t = time.time()
                while time.time() - t < 0.2:
                    stmInstance.sts3032.setMotorSpeed(
                        {deviceEnums.Side.LEFT: -50, deviceEnums.Side.RIGHT: -50}
                    )
                    if stmInstance.switch.getToggleSwitch1():
                        stmInstance.sts3032.stop()
                        return
                oldTime = time.time()
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            error = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
            deribative = regulationAngle(error - oldError)
            oldError = error
            turnSpeed = (
                mazeConstraints.TURN_P * error + mazeConstraints.TURN_D * deribative
            )
            turnSpeed = max(min(turnSpeed, 100), -100)
            turnSpeed = (
                turnSpeed if abs(turnSpeed) >= 1 else (1 if turnSpeed > 0 else -1)
            )
            (
                stmInstance.sts3032.turnRight(abs(int(turnSpeed)))
                if turnSpeed > 0
                else stmInstance.sts3032.turnLeft(abs(int(turnSpeed)))
            )
            debugPrint(
                f"gyro:{stmInstance.gyro.getValue().heading} deg, derivative: {deribative}, turnSpeed: {turnSpeed}"
            )
        stmInstance.sts3032.stop()
    debugPrint(f"Turned to heading: {stmInstance.gyro.getValue().heading} deg")


def getTurnDirection(
    fromDir: int | float, toDir: int | float
) -> mazeEnums.turnDirection:
    turnAngle = fromDir - toDir
    if turnAngle > 180:
        turnAngle -= 360
    if turnAngle < -180:
        turnAngle += 360

    if turnAngle > 0:
        return mazeEnums.turnDirection.RIGHT
    else:
        return mazeEnums.turnDirection.LEFT


def detectWall(
    lidar: ydlidar.CYdLidar,
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    points: list[LiDAR.Point] | None = None,
) -> None:
    """
    @brief LiDARのデータから壁を検出して、mapInstanceの壁情報を更新する
    @param lidar: 使用する LiDAR インスタンス
    @param mapInstance: 迷路のマップインスタンス
    @param stmInstance: 通信に使用する STM インスタンス
    @param points: LiDARのスキャンデータのリスト。Noneの場合はLiDARから取得する。
    """
    if points is None:
        points = LiDAR.getLiDARScan(lidar)
    currentDirVal = mapInstance.frontDirection.value
    stmInstance.update()
    for direction in [
        mazeEnums.absDirection.NORTH,
        mazeEnums.absDirection.EAST,
        mazeEnums.absDirection.SOUTH,
        mazeEnums.absDirection.WEST,
    ]:
        angle = (direction.value - currentDirVal + 360) % 360
        dist = LiDAR.getCertainAngleDist(angle, points)
        logger.debug(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")
        if mapInstance.getWallType()[direction] == mazeEnums.wallType.UNKNOWN:
            if (
                min(
                    stmInstance.gyro.getValue().roll,
                    360 - stmInstance.gyro.getValue().roll,
                )
                > 15
                and direction == mapInstance.frontDirection
            ):
                mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)
                logger.debug(
                    f"Detected no wall at {direction} due to high roll angle: {stmInstance.gyro.getValue().roll} deg"
                )
                continue
            if dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM:
                mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
            # elif direction == mapInstance.frontDirection: and ((LiDAR.getCertainAngleDist(0,points) - (mapInstance.arduinoNanoEvery.request_tof_distance_mm()/10 + 10)) > mazeConstraints.RAMP_TOF_THRESHOLD and LiDAR.getCertainAngleDist(0,points) < mazeConstraints.JUDGE_RAMP_LIDAR_THRESHOLD):
            #    mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
            else:
                mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)
        else:
            # 壁情報が一致してなかったらエラーとしてログに出す
            if (
                (dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM)
                and mapInstance.getWallType()[direction] != mazeEnums.wallType.WALL
            ) or (
                (dist >= mazeConstraints.WALL_DETECTION_THRESHOLD_CM)
                and mapInstance.getWallType()[direction] == mazeEnums.wallType.WALL
            ):
                logger.warning(
                    f"Inconsistent wall detection at {direction}: distance={dist} cm, map wall type={mapInstance.getWallType()[direction]}"
                )


def getVictimInfo(
    stmInstance: stm.STM,
) -> dict[deviceEnums.Side, deviceEnums.UnitVStatus]:
    """
    @brief unitv から被災者の情報を取得する
    @param stmInstance: 通信に使用する STM インスタンス
    @return: 各サイドの被災者の種類を示す辞書
    """
    stmInstance.update()
    victimData = stmInstance.unitv.getStatus()
    return victimData


def dropRescueKit(
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    victimInfo: dict[deviceEnums.Side, deviceEnums.UnitVStatus],
    side: deviceEnums.Side,
) -> None:
    """
    @brief 指定した側に救助キットを投下する
    @param stmInstance: 通信に使用する STM インスタンス
    @param side: 救助キットを投下する側
    """
    needRescueKitCount = (victimInfo[side].value - 1) % 3
    flashLED(
        stmInstance,
        mapInstance,
        5,
        0.5,
        color=[(0, 255, 0), (255, 255, 0), (255, 0, 0)][needRescueKitCount],
    )
    oppositeFlag = False
    tileColor = detectTileColor()
    firstHeading = stmInstance.gyro.getValue().heading
    if tileColor == mazeEnums.tileType.RED:
        return
    for i in range(needRescueKitCount):
        if (
            mapInstance.nowRescueKitCount[side if not oppositeFlag else side.opposite()]
            >= 1
        ):
            stmInstance.update()
            turnToCertainDirection(
                (
                    stmInstance.gyro.getValue().heading
                    + (
                        mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS * 1
                        if (i % 2) == 0
                        else mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS * -1
                    )
                )
                % 360,
                stmInstance,
            )
            mapInstance.dropRescueKit(side if not oppositeFlag else side.opposite(), 1)
            stmInstance.rescuekitservo.dropRescueKit(
                1, side if not oppositeFlag else side.opposite()
            )
            oldtime = time.time()
            while time.time() - oldtime < 0.8:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return
        elif (
            mapInstance.nowRescueKitCount[side.opposite() if not oppositeFlag else side]
            >= 1
        ):
            turnToCertainDirection(
                (
                    stmInstance.gyro.getValue().heading
                    + 180
                    + (
                        mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS * 1
                        if (i % 2) == 0
                        else mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS * -1
                    )
                )
                % 360,
                stmInstance,
            )
            mapInstance.updateFrontDirection(
                mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
            )
            mapInstance.dropRescueKit(side.opposite() if not oppositeFlag else side, 1)
            stmInstance.rescuekitservo.dropRescueKit(
                1, side.opposite() if not oppositeFlag else side
            )
            oppositeFlag = not oppositeFlag
            oldtime = time.time()
            while time.time() - oldtime < 0.8:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return
        else:
            debugPrint(f"Not enough rescue kits to drop on {side} side.")
    turnToCertainDirection(firstHeading, stmInstance)


def victimToWallType(victim: deviceEnums.UnitVStatus) -> mazeEnums.wallType:
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


def findVictimDuringMove(
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    isWallAhead: dict[deviceEnums.Side, bool],
    consequentSearchRes: dict[deviceEnums.Side, deviceEnums.UnitVStatus | None],
    getVictimDict: dict[deviceEnums.Side, defaultdict[deviceEnums.UnitVStatus, int]],
    practicalMoveTime: float,
) -> bool:
    direction = mapInstance.frontDirection
    dx = mazeEnums.directionToDelta[direction][0]
    dy = mazeEnums.directionToDelta[direction][1]
    currentPosX, currentPosY, currentPosZ = mapInstance.currentPosition

    avoidVictim = {
        s: mapInstance.getSeenVictimType(currentPosX, currentPosY, currentPosZ)[
            mazeEnums.absDirection(
                (
                    mapInstance.frontDirection.value
                    + (90 if s == deviceEnums.Side.LEFT else 270)
                )
                % 360
            )
        ]
        | mapInstance.getSeenVictimType(
            currentPosX + dx, currentPosY + dy, currentPosZ
        )[
            mazeEnums.absDirection(
                (
                    mapInstance.frontDirection.value
                    + (90 if s == deviceEnums.Side.LEFT else 270)
                )
                % 360
            )
        ]
        for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }
    # logger.debug(f"avoidVictim: {avoidVictim}")

    ####### 被災者発見処理 ######
    victimRescueFlag = False
    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        if isWallAhead[side]:  # 連続して壁の時はずっと見る
            victimInfo = stmInstance.unitv.getStatus()
            if (
                victimInfo[side] != deviceEnums.UnitVStatus.NOTHING
                and (victimInfo[side] not in avoidVictim[side])
                and consequentSearchRes[side] is None
            ):  # 被災者を発見した
                stmInstance.sts3032.stop()
                t = time.time()
                consequentSearchRes[side] = victimInfo[side]
                mapInstance.setWallType(
                    mazeEnums.absDirection(
                        (
                            mapInstance.frontDirection.value
                            + (90 if side == deviceEnums.Side.LEFT else 270)
                        )
                        % 360
                    ),
                    victimToWallType(consequentSearchRes[side]),
                )
                mapInstance.addSeenVictimType(
                    [
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + (90 if side == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ],
                    consequentSearchRes[side],
                )
                logger.info(f"Detected victim info ahead: {victimInfo}")
                dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                victimRescueFlag = True
        else:
            lastUpdateTime = stmInstance.unitv.getLastUpdateTime()[side]
            if (
                mazeConstraints.MOVE_STRAIGHT_SEC - practicalMoveTime
                < mazeConstraints.MOVE_STRAIGHT_SEC * 0.20
            ):  # 移動終了間際
                victimInfo = stmInstance.unitv.getStatus()
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    getVictimDict[side][victimInfo[side]] += 1
                    logger.debug(f"Detected victim info during movement: {victimInfo}")
            if (
                practicalMoveTime - lastUpdateTime / 1000
            ) < mazeConstraints.MOVE_STRAIGHT_SEC * 0.20:  # 移動開始直後ならば
                victimInfo = stmInstance.unitv.getStatus()
                if (  # 発見した方向に壁があり、すでに見たことのある被災者でもない　ならば
                    victimInfo[side] != deviceEnums.UnitVStatus.NOTHING
                    and mapInstance.getWallType()[
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + (90 if side == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.NO_WALL
                    and (
                        not mapInstance.isSeenVictimType(
                            [
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + (90 if side == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ],
                            victimInfo[side],
                        )
                    )
                ):

                    stmInstance.sts3032.stop()
                    t = time.time()
                    logger.info(
                        f"Detected victim info during movement needing rescue kit drop: {victimInfo}"
                    )
                    dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                    mapInstance.addSeenVictimType(
                        [
                            mazeEnums.absDirection(
                                (
                                    mapInstance.frontDirection.value
                                    + (90 if side == deviceEnums.Side.LEFT else 270)
                                )
                                % 360
                            )
                        ],
                        victimInfo[side],
                    )

                    victimRescueFlag = True
    return victimRescueFlag


def moveTile(
    direction: mazeEnums.absDirection,
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    lidar: ydlidar.CYdLidar,
    firstTime: float,
) -> tuple[bool, bool]:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    @return: (isBlackTile, stoppedByToggleSwitch)
    """

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True

    isRedTile = detectTileColor() == mazeEnums.tileType.RED

    debugPrint(
        f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}"
    )
    if direction != mapInstance.frontDirection:
        turnToCertainDirection(
            direction.value, stmInstance, rescueVictim=True, mapInstance=mapInstance
        )
    mapInstance.updateFrontDirection(direction)
    point = LiDAR.getLiDARScan(lidar)
    """
    # 坂道を壁とする処理。登れないときに使った。
    if ((LiDAR.getCertainAngleDist(0,point) - (mapInstance.arduinoNanoEvery.request_tof_distance_mm()/10 + 10)) > mazeConstraints.RAMP_TOF_THRESHOLD and LiDAR.getCertainAngleDist(0,point) < mazeConstraints.JUDGE_RAMP_LIDAR_THRESHOLD):
        mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
        return False, False
    """
    stmInstance.sts3032.stop()

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True
    points = LiDAR.getLiDARScan(lidar)
    last_scan_points = points
    dist0, dist180 = LiDAR.getCertainAngleDist([0, 180], points)
    nearestLiDARAngle = 0 if dist0 < dist180 else 180
    heading = stmInstance.gyro.getValue().heading
    oldDist = LiDAR.getCertainAngleDist(
        nearestLiDARAngle - heading + direction.value, points
    )
    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    isBigRamp = False
    isBigUpperRamp = False
    startTime = time.time()
    practicalMoveTime = 0.0
    targetSteps = (
        1  # 現在のマスから何マス先の位置まで移動するか。基本は1。坂道では伸ばす。
    )
    lastRollOnRamp = 0
    RampFinishTime = 0

    oldTime = startTime
    getVictimDict = {
        deviceEnums.Side.LEFT: defaultdict(int),
        deviceEnums.Side.RIGHT: defaultdict(int),
    }
    getTileColorDict = defaultdict(int)
    cameraBlackTileDetected = False
    isWallAhead = {
        s: LiDAR.isWallAheadTile(points, s)
        for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }

    consequentSearchRes: dict[deviceEnums.Side, deviceEnums.UnitVStatus | None] = {
        s: None for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }

    beforeDist = oldDist
    while True:
        loop_start_time = time.time()
        stmInstance.update()
        isBlackTileByCam = camera.detectTileColor() == "BLACK"
        cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return False, True

        ############ モーター制御 #############
        heading = stmInstance.gyro.getValue().heading
        roll = stmInstance.gyro.getValue().roll

        turnAngle = regulationAngle(heading - direction.value)
        leftWallDist = stmInstance.tof.getDistance()[3]
        rightWallDist = stmInstance.tof.getDistance()[1]
        gyroSteer = turnAngle * mazeConstraints.STRAIGHT_GYRO_P_GAIN
        wallSteer = 0.0

        enableDist = mazeConstraints.WALL_FOLLOW_ENABLE_DIST_CM
        targetDist = mazeConstraints.WALL_FOLLOW_TARGET_DIST_CM
        # ジャイロ誤差を最優先で減らしつつ、壁が近い場合のみ壁距離制御を足す
        if (leftWallDist <= enableDist or rightWallDist <= enableDist) and abs(
            turnAngle
        ) <= mazeConstraints.WALL_FOLLOW_GYRO_ERR_MAX_DEG:
            if leftWallDist <= enableDist and rightWallDist <= enableDist:
                wallError = rightWallDist - leftWallDist  # 両側が近いなら左右差を0へ
            elif leftWallDist <= enableDist:
                wallError = targetDist - leftWallDist  # 左が近いなら左距離を目標へ
            else:
                wallError = rightWallDist - targetDist  # 右が近いなら右距離を目標へ

            wallSteer = wallError * mazeConstraints.WALL_FOLLOW_P_GAIN
            wallSteer = max(
                min(wallSteer, mazeConstraints.WALL_FOLLOW_MAX_STEER),
                -mazeConstraints.WALL_FOLLOW_MAX_STEER,
            )

        baseLeft = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT]
        baseRight = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT]

        steer = gyroSteer + wallSteer
        leftSpeed = int(max(min(100, baseLeft + steer), -100))
        rightSpeed = int(max(min(100, baseRight - steer), -100))
        stmInstance.sts3032.setMotorSpeed(
            {deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed}
        )

        ##### 坂道判定 #####
        if min(roll, 360 - roll) > mazeConstraints.RAMP_DEG_THRESHOLD:
            isBigRamp = True
        else:
            isBigRamp = False
            
        if targetSteps > 1 and min(roll, 360 - roll) < 10 and RampFinishTime == 0:
            RampFinishTime = time.time()
            print(f"Ramp finished, RampFinishTile : {RampFinishTime }")

        ### 坂道を検出したら、平らになるまで直進する
        timeBeforeRamp = time.time()
        # if isBigRamp:
        #     nowDist = 0
        #     nowhight = 0
        #     stmInstance.sts3032.stop()
        #     time.sleep(0.5)
        #     stmInstance.sts3032.setMotorSpeed(
        #         {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50}
        #     )
        #     time.sleep(0.6)
        #     stmInstance.sts3032.stop()
        #     stmInstance.update()
        #     lastToFDist = stmInstance.tof.getDistance()[
        #         2 if stmInstance.gyro.getValue().roll < 180 else 0
        #     ]
        #     logger.info(
        #         f"Detected big ramp, lastTofDist: {lastToFDist} cm, roll: {stmInstance.gyro.getValue().roll} deg"
        #     )
        #     while (
        #         min(
        #             stmInstance.gyro.getValue().roll,
        #             360 - stmInstance.gyro.getValue().roll,
        #         )
        #         > 10
        #     ):
        #         stmInstance.update()
        #         if stmInstance.switch.getToggleSwitch1():
        #             stmInstance.sts3032.stop()
        #             return False, True

        #         if (
        #             abs(
        #                 stmInstance.tof.getDistance()[
        #                     2 if stmInstance.gyro.getValue().roll < 180 else 0
        #                 ]
        #                 - lastToFDist
        #             )
        #             > 8
        #         ):
        #             lastToFDist = stmInstance.tof.getDistance()[
        #                 2 if stmInstance.gyro.getValue().roll < 180 else 0
        #             ]
        #             continue

        #         movedDist = (
        #             stmInstance.tof.getDistance()[
        #                 2 if stmInstance.gyro.getValue().roll < 180 else 0
        #             ]
        #             - lastToFDist
        #         ) * (1 if stmInstance.gyro.getValue().roll < 180 else -1)
        #         lastToFDist = stmInstance.tof.getDistance()[
        #             2 if stmInstance.gyro.getValue().roll < 180 else 0
        #         ]
        #         nowDist += movedDist * math.cos(
        #             math.radians(stmInstance.gyro.getValue().roll)
        #         )
        #         nowhight += movedDist * math.sin(
        #             math.radians(stmInstance.gyro.getValue().roll)
        #         )
        #         stmInstance.sts3032.setMotorSpeed(
        #             {deviceEnums.Side.LEFT: 30, deviceEnums.Side.RIGHT: 30}
        #         )
        #         time.sleep(0.05)
        #         logger.debug(
        #             f"Tof:{stmInstance.tof.getDistance()[ 2 if stmInstance.gyro.getValue().roll < 180 else 0 ]}, nowDist: {nowDist:.1f} cm, nowHeight: {nowhight:.1f} cm"
        #         )
        #     # stmInstance.sts3032.stop()
        #     # time.sleep(0.5)
        #     logger.info(
        #         f"Detected big ramp, height: {nowhight:.1f} cm, distance: {nowDist:.1f} cm"
        #     )
        #     stmInstance.sts3032.setMotorSpeed(
        #         {deviceEnums.Side.LEFT: 30, deviceEnums.Side.RIGHT: 30}
        #     )
        #     time.sleep(0.15)
        #     stmInstance.sts3032.stop()
        #     time.sleep(0.5)
        #     mapInstance.setSlope(direction, nowDist, nowhight)

        ###### 黒タイル回避処理 ######
        escapeFromBlackTileRes = escapeFromBlackTile(
            stmInstance,
            mapInstance,
            direction,
            practicalMoveTime,
            cameraBlackTileDetected,
        )
        if escapeFromBlackTileRes:
            return True, False
        else:
            tempTileColor = detectTileColor()
            if tempTileColor != mazeEnums.tileType.EMPTY:
                getTileColorDict[tempTileColor] += 1
            if tempTileColor == mazeEnums.tileType.RED:
                isRedTile = True

        ###### 障害物回避処理 ######
        escapeFlag = False
        timeBeforeEscape = time.time()
        if (
            stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT]
            or stmInstance.loadcell.getPressed()[deviceEnums.Side.RIGHT]
        ):
            pressedSide = (
                deviceEnums.Side.LEFT
                if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT]
                else deviceEnums.Side.RIGHT
            )
            if stmInstance.tof.getDistance()[0] > 20:
                escapeFromObstacle(pressedSide, stmInstance)
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
                logger.debug(
                    f"Obstacle escape adjustment: {(time.time() - oldTime) * np.cos(np.radians(abs(stmInstance.gyro.getValue().roll))) * 0.1}"
                )
                practicalMoveTime -= 0.1
                escapeFlag = True
                time.sleep(0.1)

        ###### 1マス移動完了判定（時間制御） ######
        if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC * targetSteps:
            if isBigRamp:
                targetSteps += 1
                lastRollOnRamp = roll
                continue

            # print(time.time()-RampFinishTime)
            # if targetSteps > 1 and ((time.time() - RampFinishTime) < mazeConstraints.MOVE_STRAIGHT_SEC * 0.5):
            #     time.sleep(mazeConstraints.MOVE_STRAIGHT_SEC * 0.5 - (time.time() - RampFinishTime))
            stmInstance.sts3032.stop()
            break
        if (
            stmInstance.tof.getDistance()[0]
            < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM
            and not isBigRamp
        ):
            # 坂でなく、前方の壁までの距離が小さくなったときは止める。
            stmInstance.sts3032.stop()
            break

        ####### 被災者発見処理 ######
        victimRescueFlag = findVictimDuringMove(
            mapInstance,
            stmInstance,
            isWallAhead,
            consequentSearchRes,
            getVictimDict,
            practicalMoveTime,
        )

        if not victimRescueFlag:
            pratical_loop_time = 0
            if escapeFlag:
                pratical_loop_time = timeBeforeEscape - oldTime
            elif isBigRamp:
                pratical_loop_time = timeBeforeRamp - oldTime
            else:
                pratical_loop_time = time.time() - oldTime

            practicalMoveTime += (
                pratical_loop_time
                * np.cos(np.radians(abs(roll)))
                * (1 if roll > 180 else 0.9)
            )

        debugPrint(
            f"roll: {stmInstance.gyro.getValue().roll} deg, practicalMoveTime: {practicalMoveTime} sec"
        )
        oldTime = time.time()
        logger.debug(
            f"moveTile loop time: {(time.time() - loop_start_time) * 1000:.1f} ms"
        )

    ###### 移動後、目の前が壁であれば位置調整のため少し前進 ######
    pts = LiDAR.getLiDARScan(lidar)
    if 15 < LiDAR.getCertainAngleDist(-heading + direction.value, pts) < 27:
        stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")
        while True:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return False, True
            scanPoints = LiDAR.getLiDARScan(lidar)
            last_scan_points = scanPoints
            heading = stmInstance.gyro.getValue().heading
            currentDist = LiDAR.getCertainAngleDist(
                -heading + direction.value, scanPoints
            )
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
    stmInstance.sts3032.stop()

    ##### 上り坂をのぼった後の被災者検知 #####
    oldTime = time.time()
    # if isBigUpperRamp and (not isBigRamp):
    #     turnToCertainDirection((direction.value + 30) % 360, stmInstance)
    #     t = time.time()
    #     while time.time() - t < 0.5:
    #         stmInstance.update()
    #         unitvStatus = stmInstance.unitv.getStatus()
    #         for s in deviceEnums.Side:
    #             if unitvStatus[s] != deviceEnums.UnitVStatus.NOTHING:
    #                 getVictimDict[s][unitvStatus[s]] += 1
    #                 logger.debug(
    #                     f"Detected victim info during big upper ramp movement: {unitvStatus}"
    #                 )
    #     turnToCertainDirection((direction.value - 30) % 360, stmInstance)
    #     t = time.time()
    #     while time.time() - t < 0.5:
    #         stmInstance.update()
    #         unitvStatus = stmInstance.unitv.getStatus()
    #         for s in deviceEnums.Side:
    #             if unitvStatus[s] != deviceEnums.UnitVStatus.NOTHING:
    #                 getVictimDict[s][unitvStatus[s]] += 1
    #                 logger.debug(
    #                     f"Detected victim info during big upper ramp movement: {unitvStatus}"
    #                 )
    #     turnToCertainDirection(direction.value, stmInstance)

    ##### 坂を上ったのであればマップに登録 #####
    if targetSteps > 1:
        if targetSteps == 2:   
            mapInstance.setSlope(
                direction,
                (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM,
                (targetSteps - 1) * 15 * (1 if lastRollOnRamp < 180 else -1),
            )
        else:
            mapInstance.setSlope(
                direction,
                (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM,
                (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM * math.tan(math.radians(lastRollOnRamp)),
            )
    mapInstance.moveTo(direction)
    detectWall(lidar, mapInstance, stmInstance)

    ##### 移動終了時の被災者検出処理 #####
    ##### 移動終了直前、移動終了時に見たもの、上り坂をのぼったあとの特殊処理で見たものについて、ここで救助動作を行う。
    maxVictimInfo = {
        side: (
            max(getVictimDict[side], key=getVictimDict[side].get)
            if getVictimDict[side]
            else deviceEnums.UnitVStatus.NOTHING
        )
        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }
    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        if (
            mapInstance.getWallType()[
                mazeEnums.absDirection(
                    (
                        mapInstance.frontDirection.value
                        + (90 if side == deviceEnums.Side.LEFT else 270)
                    )
                    % 360
                )
            ]
            == mazeEnums.wallType.NO_WALL
        ):
            continue
        if not (consequentSearchRes[side] is None):
            mapInstance.addSeenVictimType(
                [
                    mazeEnums.absDirection(
                        (
                            mapInstance.frontDirection.value
                            + (90 if side == deviceEnums.Side.LEFT else 270)
                        )
                        % 360
                    )
                ],
                consequentSearchRes[side],
            )
        elif (
            mapInstance.isSeenVictimType(
                [
                    mazeEnums.absDirection(
                        (
                            mapInstance.frontDirection.value
                            + (90 if side == deviceEnums.Side.LEFT else 270)
                        )
                        % 360
                    )
                ],
                maxVictimInfo[side],
            )
            == False
            and maxVictimInfo[side] != deviceEnums.UnitVStatus.NOTHING
            and mapInstance.getWallType()[
                mazeEnums.absDirection(
                    (
                        mapInstance.frontDirection.value
                        + (90 if side == deviceEnums.Side.LEFT else 270)
                    )
                    % 360
                )
            ]
            == mazeEnums.wallType.WALL
        ):

            mapInstance.addSeenVictimType(
                [
                    mazeEnums.absDirection(
                        (
                            mapInstance.frontDirection.value
                            + (90 if side == deviceEnums.Side.LEFT else 270)
                        )
                        % 360
                    )
                ],
                maxVictimInfo[side],
            )

            logger.info(f"Decided victim on {side} side: {maxVictimInfo[side]}")
            dropRescueKit(stmInstance, mapInstance, maxVictimInfo, side)

    ###### 銀・青タイル判別処理 ######
    tileType = mazeEnums.tileType.EMPTY
    nowMaxCount = 0
    if isSilverTile():
        stmInstance.buzzer.playMusic(buzzerSongs.checkpoint)
        mapInstance.setTileType(mazeEnums.tileType.SILVER)
        tileType = mazeEnums.tileType.SILVER
    else:
        for t in list(getTileColorDict.keys()):
            if t.value == "E":
                continue
            if (
                getTileColorDict[t] > getTileColorDict[tileType]
                and getTileColorDict[t]
                >= max(mazeConstraints.MIN_TILE_DETECTION_THRESHOLD, nowMaxCount)
                and t != mazeEnums.tileType.EMPTY
            ):
                tileType = t
                nowMaxCount = getTileColorDict[t]
        mapInstance.setTileType(
            tileType
            if tileType != mazeEnums.tileType.BLACK
            else mazeEnums.tileType.EMPTY
        )
    logger.info(
        f"Tile color detection counts: {dict(getTileColorDict)}, decided tile type: {tileType}"
    )

    if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
        stmInstance.buzzer.playMusic(buzzerSongs.swamp)
        time.sleep(5.2)
    if mapInstance.getTileType() != mazeEnums.tileType.EMPTY:
        logger.info(
            f"Moved to {mapInstance.currentPosition}, Tile type: {mapInstance.getTileType()}, Wall types: {mapInstance.getWallType()}"
        )
    stmInstance.sts3032.stop()
    return False, False


def moveNextTile(
    direction: mazeEnums.absDirection,
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    lidar: ydlidar.CYdLidar,
) -> tuple[bool, bool]:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    @return: isBlackTile, stoppedByToggleSwitch
    """
    firstTime = time.time()
    isBlack, stopped = moveTile(direction, mapInstance, stmInstance, lidar, firstTime)

    if not isBlack:
        stmInstance.sts3032.stop()

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return isBlack, True
    return isBlack, stopped
