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
DEBUG_TIMING_LOG = True

lastDist = {
    mazeEnums.absDirection.NORTH: 0,
    mazeEnums.absDirection.EAST: 0,
    mazeEnums.absDirection.SOUTH: 0,
    mazeEnums.absDirection.WEST: 0,
}


def turnOnLED(stmInstance: stm.STM, color: tuple[int, int, int]) -> None:
    """
    @brief LEDを点灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(*color)
    stmInstance.rearSTM.victimled(color)


def turnOffLED(stmInstance: stm.STM) -> None:
    """
    @brief LEDを消灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(0, 0, 0)
    stmInstance.rearSTM.victimled((0, 0, 0))


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
        logger.debug(" ".join(str(m) for m in message))


def debugTimingPrint(label: str, start_time: float | None = None) -> float:
    if not (mazeConstraints.DEBUG_MODE and DEBUG_TIMING_LOG):
        return time.perf_counter()

    current_time = time.perf_counter()
    if start_time is None:
        logger.debug(f"[{current_time:.6f}] {label}")
    else:
        logger.debug(
            f"[{current_time:.6f}] {label} (+{(current_time - start_time) * 1000:.1f} ms)"
        )
    return current_time


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
            return mazeEnums.tileType.EMPTY
        r, g, b = colorSensor._colorRGB
        rf1, rf2 = colorSensor._reflectance
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
        elif (rf1 <= mazeConstraints.SILVERTILE_REFLECTANCE_THRESHOLD_RF1) or (
            rf2 <= mazeConstraints.SILVERTILE_REFLECTANCE_THRESHOLD_RF2
        ):
            return mazeEnums.tileType.SILVER
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
    time.sleep(0.3)
    if deviceEnumsSide == deviceEnums.Side.LEFT:
        stmInstance.sts3032.turnRight(40)
        time.sleep(0.3)
    else:
        stmInstance.sts3032.turnLeft(40)
        time.sleep(0.3)
    stmInstance.sts3032.stop()


def escapeFromBlackTile(
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    direction: mazeEnums.absDirection,
    practicalMoveTime: float,
    cameraBlackTileDetected: bool,
    getVictimDict: dict[deviceEnums.Side, defaultdict[deviceEnums.UnitVStatus, int]]
) -> bool:
    tempTileColor = detectTileColor()
    if tempTileColor == mazeEnums.tileType.BLACK and cameraBlackTileDetected:
        stmInstance.sts3032.stop()
        mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
        logger.info("Black tile detected! Stopping movement. Starting escape maneuver.")
        stmInstance.sts3032.setMotorSpeed(
            {deviceEnums.Side.LEFT: -100, deviceEnums.Side.RIGHT: -100}
        )
        startEscapeTime = time.time()
        while time.time() - startEscapeTime < practicalMoveTime + 0.16:  # 0.16は補正値
            stmInstance.update()
            victimInfo = stmInstance.unitv.getStatus()
            for side in deviceEnums.Side:
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    getVictimDict[side][victimInfo[side]] += 1
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return True
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
    normalTurnSpeed: int = mazeConstraints.TURN_SPD,
    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] | None = None,
) -> None:
    """
    @brief 指定した絶対方向に向く
    @param targetDir: 目標の絶対方向 (0-359)
    @param stmInstance: 通信に使用する STM インスタンス
    @param rescueVictim: 回転中に被災者を救助するかどうか
    @param mapInstance: 迷路のマップインスタンス。rescueVictimがTrueのときには必要
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
            stmInstance.sts3032.turnRight(normalTurnSpeed)
            if turnDirection == mazeEnums.turnDirection.RIGHT
            else stmInstance.sts3032.turnLeft(normalTurnSpeed)
        )
        debugPrint(
            "current heading:",
            stmInstance.gyro.getValue().heading,
            "target:",
            targetDir,
        )
        oldTurnDir = turnDirection
        rescueStopped = False
        _turn_start_time = time.time()
        while (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            > mazeConstraints.TURN_THRESHOLD_DEG
        ):
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            _before_rescue_time = time.time()
            if rescueVictim:
                for s in deviceEnums.Side:
                    victimInfo = getVictimInfo(stmInstance)
                    tiletype = detectTileColor()
                    if (
                        victimInfo[s] != deviceEnums.UnitVStatus.NOTHING
                        and mapInstance.isSeenVictimType(  # type: ignore
                            getQuantizedDir(
                                stmInstance.gyro.getValue().heading
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            ),
                            victimInfo[s],
                        )
                        == False
                        and mapInstance.getWallType()[  # type: ignore
                            getQuantizedDir(
                                stmInstance.gyro.getValue().heading
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )[0]
                        ]
                        == mazeEnums.wallType.WALL
                        and mapInstance.getWallType()[  # type: ignore
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

                            dropRescueKit(stmInstance, mapInstance, victimInfo, s, detectedVictimDuringMove)  # type: ignore
                            mapInstance.addSeenVictimType(  # type: ignore
                                getQuantizedDir(
                                    stmInstance.gyro.getValue().heading
                                    + (90 if s == deviceEnums.Side.LEFT else 270)
                                ),
                                victimInfo[s],
                            )
                            logger.info(
                                f"Dropped rescue kit, detected victim info: {victimInfo}"
                            )
            _turn_start_time += time.time() - _before_rescue_time
            turnDirection = getTurnDirection(
                stmInstance.gyro.getValue().heading, targetDir
            )
            if rescueStopped or turnDirection != oldTurnDir:
                (
                    stmInstance.sts3032.turnRight(normalTurnSpeed)
                    if turnDirection == mazeEnums.turnDirection.RIGHT
                    else stmInstance.sts3032.turnLeft(normalTurnSpeed)
                )
                oldTurnDir = turnDirection
                rescueStopped = False
            if mazeConstraints.USE_STUCK_AVOIDANCE_WHEN_TURNING and (
                time.time() - _turn_start_time
                > mazeConstraints.STUCK_AVOIDANCE_WHEN_TURNING_THRESHOLD_SEC
            ):
                stmInstance.sts3032.stop()
                logger.warning(
                    "Stuck avoidance triggered during turning. Performing escape maneuver."
                )
                stmInstance.sts3032.setMotorSpeed(
                    mazeConstraints.STUCK_AVOIDANCE_WHEN_TURNING_FORWARD_SPEED
                )
                time.sleep(
                    mazeConstraints.STUCK_AVOIDANCE_WHEN_TURNING_FORWARD_TIME_SEC
                )
                stmInstance.sts3032.stop()
                _turn_start_time = time.time()

                (
                    stmInstance.sts3032.turnRight(normalTurnSpeed)
                    if turnDirection == mazeEnums.turnDirection.RIGHT
                    else stmInstance.sts3032.turnLeft(normalTurnSpeed)
                )

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
            stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD_SLOW)
            if oldTurnDir == mazeEnums.turnDirection.RIGHT
            else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD_SLOW)
        )
        while (
            abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir))
            > mazeConstraints.TURN_THRESHOLD_DEG_FIX
        ):
            if oldTurnDir != getTurnDirection(
                stmInstance.gyro.getValue().heading, targetDir
            ):
                (
                    stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD_SLOW)
                    if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
                    == mazeEnums.turnDirection.RIGHT
                    else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD_SLOW)
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
    points: LiDAR.ScanMap | None = None,
    enableOverwrite: bool = False,
) -> bool:
    """
    @brief LiDARのデータから壁を検出して、mapInstanceの壁情報を更新する
    @param lidar: 使用する LiDAR インスタンス
    @param mapInstance: 迷路のマップインスタンス
    @param stmInstance: 通信に使用する STM インスタンス
    @param points: LiDARのスキャンデータのリスト。Noneの場合はLiDARから取得する。
    """
    if enableOverwrite:
        logger.debug("overwrite the wall data")
    if points is None:
        points = LiDAR.getLiDARScan(lidar)
    currentDirVal = mapInstance.frontDirection.value
    stmInstance.update()
    stmInstance.frontTSD10.update()
    for direction in [
        mazeEnums.absDirection.NORTH,
        mazeEnums.absDirection.EAST,
        mazeEnums.absDirection.SOUTH,
        mazeEnums.absDirection.WEST,
    ]:
        angle = (direction.value - currentDirVal + 360) % 360
        dist = LiDAR.getCertainAngleDist(angle, points)
        logger.debug(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")
        if (
            mapInstance.getWallType()[direction] == mazeEnums.wallType.UNKNOWN
            or enableOverwrite
        ):
            if mazeConstraints.USE_OBSTACLE_DETECTION_MODE_WHEN_DETECTING_WALL and (
                angle == 0
                or not mazeConstraints.USE_OBSTACLE_DETECTION_MODE_WHEN_DETECTING_WALL_ONLY_FRONT
            ):
                detectedWallStatus = LiDAR.judgeWallCertainAngle(angle, points)
                logger.info(f"angle: {angle}, result: {detectedWallStatus}")
                if detectedWallStatus == deviceEnums.judgeWallResult.WALL:
                    if angle == 0:  # 正面に壁がある時は坂道判定を入れる
                        logger.debug(
                            f"Front LiDAR distance: {dist} cm, TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm"
                        )
                        if (
                            dist - stmInstance.frontTSD10.get_distance() / 10
                        ) > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM:
                            mapInstance.setWallType(
                                direction, mazeEnums.wallType.NO_WALL
                            )
                            logger.debug(
                                f"Detected ramp at {direction} due to TSD10 distance: {stmInstance.frontTSD10.get_distance()} mm"
                            )
                            continue
                    mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
                elif detectedWallStatus == deviceEnums.judgeWallResult.CENTER_OBSTACLE:
                    if (
                        dist - stmInstance.frontTSD10.get_distance() / 10
                    ) > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM:
                        # 階段は測定結果が荒れてCENTER_OBSTACLEと判断してしまうが、ToFの値の差で坂と判断できる
                        mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)
                    else:
                        mapInstance.setWallType(
                            direction, mazeEnums.wallType.OBSTACLE_WALL
                        )
                        logger.debug(
                            f"Detected center obstacle at {direction}, distance: {dist} cm"
                        )
                # elif detectedWallStatus == deviceEnums.judgeWallResult.LEFT_OBSTACLE or detectedWallStatus == deviceEnums.judgeWallResult.RIGHT_OBSTACLE:
                #     mapInstance.setWallType(
                #         direction, mazeEnums.wallType.SIDE_OBSTACLE_WALL
                #     )
                #     logger.debug(
                #         f"Detected side obstacle at {direction}, distance: {dist} cm"
                #     )
                else:
                    mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)
            else:
                if (  # 坂道上では前の壁は検出しない
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
                    mapInstance.isSlopeDetected = True
                    continue

                if dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM:
                    mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
                else:
                    mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

                if angle == 0:
                    logger.debug(
                        f"Front LiDAR distance: {dist} cm, TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm"
                    )
                    if (
                        dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM
                        and (dist - stmInstance.frontTSD10.get_distance() / 10)
                        > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM
                    ):
                        mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)
                        logger.debug(
                            f"Detected ramp at {direction} due to TSD10 distance: {stmInstance.frontTSD10.get_distance()} mm"
                        )
        else:
            if mazeConstraints.USE_OBSTACLE_DETECTION_MODE_WHEN_DETECTING_WALL and (
                angle == 0
                or not mazeConstraints.USE_OBSTACLE_DETECTION_MODE_WHEN_DETECTING_WALL_ONLY_FRONT
            ):
                detectedWallStatus = LiDAR.judgeWallCertainAngle(angle, points)
                if detectedWallStatus == deviceEnums.judgeWallResult.WALL:
                    if angle == 0:  # 正面に壁がある時は坂道判定を入れる
                        logger.debug(
                            f"Front LiDAR distance: {dist} cm, TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm"
                        )
                        if (
                            dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM
                            and (dist - stmInstance.frontTSD10.get_distance() / 10)
                            > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM
                        ):
                            if (
                                mapInstance.getWallType()[direction]
                                != mazeEnums.wallType.NO_WALL
                            ):
                                if not mazeConstraints.DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR:
                                    mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL )
                                logger.warning(
                                    f"inconsistent wall detection: LiDAR indicates ramp, but map indicates wall. distance: {dist} cm, TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm"
                                )
                                return False
                            continue
                    if (
                        mapInstance.getWallType()[direction]
                        == mazeEnums.wallType.NO_WALL
                    ):
                        if not mazeConstraints.DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR:
                            mapInstance.setWallType(direction, mazeEnums.wallType.WALL )
                        logger.warning(
                            f"inconsistent wall detection: LiDAR indicates wall, but map indicates no wall. distance: {dist} cm"
                        )
                        return False
                #### CENTER_OBSTACLEとNO_WALLは判定が変わる可能性があるので、その差は許容
                elif detectedWallStatus == deviceEnums.judgeWallResult.CENTER_OBSTACLE:
                    # 坂でないか判断
                    if (
                        dist - stmInstance.frontTSD10.get_distance() / 10
                    ) > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM:  # このとき坂です
                        if (
                            mapInstance.getWallType()[direction]
                            != mazeEnums.wallType.NO_WALL
                        ):
                            logger.warning(
                                f"inconsistent wall detection: LiDAR indicates ramp, but map indicates wall. distance: {dist} cm, TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm"
                            )
                            return False
                    if (
                        mapInstance.getWallType()[direction]
                        != mazeEnums.wallType.OBSTACLE_WALL
                        and mapInstance.getWallType()[direction]
                        != mazeEnums.wallType.NO_WALL
                    ):
                        logger.warning(
                            f"inconsistent wall detection: LiDAR indicates center obstacle, but map indicates {mapInstance.getWallType()[direction]}. distance: {dist} cm"
                        )
                        return False
                    mapInstance.setWallType(direction, mazeEnums.wallType.OBSTACLE_WALL)
                    logger.debug(
                        f"Detected center obstacle at {direction}, distance: {dist} cm"
                    )
                else:
                    if (
                        mapInstance.getWallType()[direction]
                        != mazeEnums.wallType.NO_WALL
                        and mapInstance.getWallType()[direction]
                        != mazeEnums.wallType.OBSTACLE_WALL
                    ):
                        if not mazeConstraints.DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR:
                            mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL )
                        logger.warning(
                            f"inconsistent wall detection: LiDAR indicates no wall, but map indicates {mapInstance.getWallType()[direction]}. distance: {dist} cm"
                        )
                        return False
            else:
                # 壁情報が一致してなかったらエラーとしてログに出す
                if (
                    (dist < mazeConstraints.WALL_DETECTION_THRESHOLD_CM)
                    and (
                        not (
                            direction == mapInstance.frontDirection
                            and (dist - stmInstance.frontTSD10.get_distance() / 10)
                            < mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM
                        )
                    )
                    and mapInstance.getWallType()[direction]
                    == mazeEnums.wallType.NO_WALL
                ) or (
                    (dist >= mazeConstraints.WALL_DETECTION_THRESHOLD_CM)
                    and mapInstance.getWallType()[direction]
                    != mazeEnums.wallType.NO_WALL
                    and mapInstance.getWallType()[direction]
                    != mazeEnums.wallType.OBSTACLE_WALL
                ):
                    logger.warning(
                        f"Inconsistent wall detection at {direction}: distance={dist} cm, map wall type={mapInstance.getWallType()[direction]}"
                    )
                    if (
                        not mazeConstraints.DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR_ONLY_FRONT
                    ):
                        return False
                    elif direction == mapInstance.frontDirection:
                        return False
    return True


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
    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] | None = None,
) -> None:
    """
    @brief 指定した側に救助キットを投下する
    @param stmInstance: 通信に使用する STM インスタンス
    @param side: 救助キットを投下する側
    """
    needRescueKitCount = (victimInfo[side].value - 1) % 3

    flashLED_flag = True

    # Cognitive Targetの被災者に対しては、2点でもキット1つしか投下しないルールを適用（誤検知による過剰投下が怖いので）
    if (
        mazeConstraints.DROP_ONLY_ONE_KIT_FOR_HARMED_COGNITIVE
        and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM
    ):
        needRescueKitCount = 1
        logger.info(
            "Detected 2-point Cognitive Target victim, dropping only one rescue kit."
        )

    # キット2つ必要な被災者を救助するとき、もしもキット1つの被災者を先に検出していた場合は、LEDを光らせず、キットを1つだけ落とす。
    # currentPosX, currentPosY, currentPosZ = mapInstance.currentPosition

    # avoidVictim: dict[deviceEnums.Side, set[deviceEnums.UnitVStatus]] = {
    #     s: mapInstance.getSeenVictimType(currentPosX, currentPosY, currentPosZ)[
    #         mazeEnums.absDirection(
    #             (
    #                 mapInstance.frontDirection.value
    #                 + (90 if s == deviceEnums.Side.LEFT else 270)
    #             )
    #             % 360
    #         )
    #     ]
    #     for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    # }
    if detectedVictimDuringMove is not None and len(detectedVictimDuringMove) > 0:
        maxDroppedKit = max(
            [
                (v.value - 1) % 3
                for v in detectedVictimDuringMove
                if v != deviceEnums.UnitVStatus.NOTHING
            ],
            default=0,
        )
        logger.info(
            f"Detected previously seen victims: {detectedVictimDuringMove}, max dropped kits: {maxDroppedKit}. Adjusting needRescueKitCount accordingly."
        )
        needRescueKitCount = max(needRescueKitCount - maxDroppedKit, 0)
        flashLED_flag = False
    if detectedVictimDuringMove is not None:
        detectedVictimDuringMove.add(victimInfo[side])

    ############################

    if flashLED_flag:
        stmInstance.rearSTM.playMusic(buzzerSongs.detectedVictim)
        if mazeConstraints.USE_SAME_COLOR_FOR_VICTIM_DETECTION_LED_BLINK:
            flashLED(
                stmInstance,
                mapInstance,
                5,
                0.5,
                color=(255, 255, 255),
            )
        else:
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
    if tileColor == mazeEnums.tileType.RED:  # 床色が赤ならレスキューキットを落とさない
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
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
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
            if not mazeConstraints.TURN_180_WHEN_LACK_OF_KIT:
                debugPrint("Lack of rescue kits and 180-degree turn disabled.")
                continue
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
                rescueVictim=False,
                mapInstance=mapInstance,
                normalTurnSpeed=30
            )
            # mapInstance.updateFrontDirection(
            #     mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
            # )
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
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
    turnToCertainDirection(firstHeading, stmInstance, normalTurnSpeed=30)


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
    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] | None = None,
    useBackwardMove: bool = True,
) -> bool:
    direction = mapInstance.frontDirection
    dx = mazeEnums.directionToDelta[direction][0]
    dy = mazeEnums.directionToDelta[direction][1]
    currentPosX, currentPosY, currentPosZ = mapInstance.currentPosition

    avoidVictim: dict[deviceEnums.Side, set[deviceEnums.UnitVStatus]] = {
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
                stmInstance.update()
                _back_is_wall = stmInstance.tof.getDistance()[3] < 21
                
                # 行き過ぎてしまうことが多いので、少し下がる
                if mazeConstraints.BACKWARD_AFTER_DROP_KIT and useBackwardMove and not _back_is_wall:
                    logger.info("Moving backward before dropping rescue kit")
                    stmInstance.sts3032.setMotorSpeed(
                        mazeConstraints.GO_BACKWARD_LOW_SPEED
                    )
                    time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                    stmInstance.sts3032.stop()

                t = time.time()

                logger.info(f"Detected victim info ahead: {victimInfo}")
                dropRescueKit(
                    stmInstance, mapInstance, victimInfo, side, detectedVictimDuringMove
                )
                consequentSearchRes[side] = victimInfo[side]
                mapInstance.setWallType(
                    mazeEnums.absDirection(
                        (
                            mapInstance.frontDirection.value
                            + (90 if side == deviceEnums.Side.LEFT else 270)
                        )
                        % 360
                    ),
                    victimToWallType(consequentSearchRes[side]),  # type: ignore
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
                    consequentSearchRes[side],  # type: ignore
                )

                # 下がった分前進して元の位置に戻る
                if mazeConstraints.BACKWARD_AFTER_DROP_KIT and useBackwardMove and not _back_is_wall:
                    stmInstance.sts3032.setMotorSpeed(
                        mazeConstraints.GO_STRAIGHT_LOW_SPEED
                    )
                    time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                    stmInstance.sts3032.stop()

                victimRescueFlag = True
        else:
            lastUpdateTime = stmInstance.unitv.getLastUpdateTime()[side]
            if (
                mazeConstraints.MOVE_STRAIGHT_SEC - practicalMoveTime
                < mazeConstraints.MOVE_STRAIGHT_SEC * 0.40
            ):  # 移動終了間際
                victimInfo = stmInstance.unitv.getStatus()
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    getVictimDict[side][victimInfo[side]] += 1
                    logger.debug(f"Detected victim info during movement: {victimInfo}")

            if (
                practicalMoveTime - lastUpdateTime / 1000  # type: ignore
            ) < mazeConstraints.MOVE_STRAIGHT_SEC * 0.30:  # 移動開始直後ならば
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
                    and mapInstance.getWallType()[
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + (90 if side == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.OBSTACLE_WALL
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
                    stmInstance.update()
                    _back_is_wall = stmInstance.tof.getDistance()[3] < 21
                    
                    if mazeConstraints.BACKWARD_AFTER_DROP_KIT and useBackwardMove and not _back_is_wall:
                        logger.info("Moving backward before dropping rescue kit")
                        stmInstance.sts3032.setMotorSpeed(
                            mazeConstraints.GO_BACKWARD_LOW_SPEED
                        )
                        time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                        stmInstance.sts3032.stop()
                    dropRescueKit(
                        stmInstance,
                        mapInstance,
                        victimInfo,
                        side,
                        detectedVictimDuringMove,
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
                        victimInfo[side],
                    )
                    if mazeConstraints.BACKWARD_AFTER_DROP_KIT and useBackwardMove and not _back_is_wall:
                        stmInstance.sts3032.setMotorSpeed(
                            mazeConstraints.GO_STRAIGHT_LOW_SPEED
                        )
                        time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                        stmInstance.sts3032.stop()

                    victimRescueFlag = True
    return victimRescueFlag


def turnWith45VictimCheck(
    targetDir: mazeEnums.absDirection,
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] | None = None,
):
    """
    @brief 回転する際に45度で止まって被災者を確認する関数
    @param targetDir: 目標の絶対方向 (0-359)
    """

    turnDir = (
        targetDir.value - mapInstance.frontDirection.value + 360
    ) % 360  # 回転する角度

    cnt = 1
    if turnDir == 180:
        cnt = 2
        turnDir = 90
    cnt2_45_skip = False
    for _c in range(cnt):
        if turnDir == 90 or turnDir == 270:
            if turnDir == 270:
                turnDir = -90
            # 前と右に壁があるなら、途中で止まる

            watchVictimFlag = {  # 45度回転後に見るか
                deviceEnums.Side.LEFT: False,
                deviceEnums.Side.RIGHT: False,
            }
            for s in deviceEnums.Side:
                if (
                    mapInstance.getWallType()[  # 回転する前
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + turnDir * _c
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.NO_WALL
                    and mapInstance.getWallType()[  # 回転した後
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + turnDir * (_c + 1)
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.NO_WALL
                ):
                    watchVictimFlag[s] = True

            if (not watchVictimFlag[deviceEnums.Side.LEFT]) and (
                not watchVictimFlag[deviceEnums.Side.RIGHT]
            ):
                if cnt == 2 and _c == 0:
                    for (
                        s
                    ) in deviceEnums.Side:  # 次の回転で壁を見るひつようがあるかも見る
                        if (
                            mapInstance.getWallType()[  # 回転する前
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 1)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.NO_WALL
                            and mapInstance.getWallType()[  # 回転する前
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 1)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.OBSTACLE_WALL
                            and mapInstance.getWallType()[  # 回転した後
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 2)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.NO_WALL
                            and mapInstance.getWallType()[  # 回転した後
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 2)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.OBSTACLE_WALL
                        ):
                            watchVictimFlag[s] = True
                    if (
                        watchVictimFlag[deviceEnums.Side.LEFT]
                        or watchVictimFlag[deviceEnums.Side.RIGHT]
                    ):
                        turnToCertainDirection(  # 135度で止まる
                            (
                                mapInstance.frontDirection.value
                                + turnDir * (_c + 1)
                                + turnDir // 2
                            )
                            % 360,
                            stmInstance,
                            rescueVictim=False,
                            mapInstance=mapInstance,
                            detectedVictimDuringMove=detectedVictimDuringMove,
                        )
                        cnt2_45_skip = True
                        continue
                    else:
                        # 180度で止まる
                        turnToCertainDirection(
                            (mapInstance.frontDirection.value + turnDir * (_c + 2))
                            % 360,
                            stmInstance,
                            rescueVictim=False,
                            mapInstance=mapInstance,
                            detectedVictimDuringMove=detectedVictimDuringMove,
                        )
                        return
                else:
                    turnToCertainDirection(
                        (mapInstance.frontDirection.value + turnDir * (_c + 1)) % 360,
                        stmInstance,
                        rescueVictim=False,
                        mapInstance=mapInstance,
                        detectedVictimDuringMove=detectedVictimDuringMove,
                    )
                continue

            # まず45度周る
            if not cnt2_45_skip:
                turnToCertainDirection(
                    (mapInstance.frontDirection.value + turnDir * _c + turnDir // 2)
                    % 360,
                    stmInstance,
                    rescueVictim=True,
                    mapInstance=mapInstance,
                    detectedVictimDuringMove=detectedVictimDuringMove,
                )

            # 被災者を確認する
            startTime = time.time()
            stmInstance.unitv.set45Mode(True)
            while (
                time.time() - startTime
                < mazeConstraints.DETECT_VICTIM_45_CHECK_TIME_SEC
            ):
                stmInstance.update()
                victimInfo = getVictimInfo(stmInstance)
                for s in deviceEnums.Side:
                    #  見るべき方向に被災者がいて、かつ見たことのない被災者ならば救助キットを落とす
                    if (
                        watchVictimFlag[s]
                        and victimInfo[s] != deviceEnums.UnitVStatus.NOTHING
                        and not mapInstance.isSeenVictimType(
                            [
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * _c
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                ),
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 1)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                ),
                            ],
                            victimInfo[s],
                        )
                    ):
                        logger.info(
                            f"Find victim on {'LEFT' if s == deviceEnums.Side.LEFT else 'RIGHT'} side during 45-degree turn check: {victimInfo[s]}"
                        )
                        dropRescueKit(
                            stmInstance,
                            mapInstance,
                            victimInfo,
                            s,
                            detectedVictimDuringMove,
                        )
                        mapInstance.addSeenVictimType(
                            [
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * _c
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                ),
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 1)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                ),
                            ],
                            victimInfo[s],
                        )
                        logger.info(
                            f"Dropped rescue kit, detected victim info: {victimInfo}"
                        )

                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return

            stmInstance.unitv.set45Mode(False)

            # もう45度回る
            turnToCertainDirection(
                (mapInstance.frontDirection.value + turnDir * (_c + 1)) % 360,
                stmInstance,
                rescueVictim=True,
                mapInstance=mapInstance,
                detectedVictimDuringMove=detectedVictimDuringMove,
            )


def turnWithSlowVictimCheck(
    targetDir: mazeEnums.absDirection,
    stmInstance: stm.STM,
    mapInstance: mazeMap.mazeMap,
    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] | None = None,
):
    """
    @brief 回転する際にゆっくり回って被災者を確認する関数
    @param targetDir: 目標の絶対方向 (0-359)
    """
    turnDir = (
        targetDir.value - mapInstance.frontDirection.value + 360
    ) % 360  # 回転する角度

    cnt = 1
    if turnDir == 180:
        cnt = 2
        turnDir = 90
    for _c in range(cnt):
        if turnDir == 90 or turnDir == 270:
            if turnDir == 270:
                turnDir = -90
            # 前と右に壁があるなら、途中で止まる

            watchVictimFlag = {  # 45度回転後に見るか
                deviceEnums.Side.LEFT: False,
                deviceEnums.Side.RIGHT: False,
            }
            for s in deviceEnums.Side:
                if (
                    mapInstance.getWallType()[  # 回転する前
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + turnDir * _c
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.NO_WALL
                    and mapInstance.getWallType()[  # 回転した後
                        mazeEnums.absDirection(
                            (
                                mapInstance.frontDirection.value
                                + turnDir * (_c + 1)
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        )
                    ]
                    != mazeEnums.wallType.NO_WALL
                ):
                    watchVictimFlag[s] = True

            if (not watchVictimFlag[deviceEnums.Side.LEFT]) and (
                not watchVictimFlag[deviceEnums.Side.RIGHT]
            ):  # 45度で止まる必要がない場合
                if cnt == 2 and _c == 0:
                    for (
                        s
                    ) in deviceEnums.Side:  # 次の回転で壁を見るひつようがあるかも見る
                        if (
                            mapInstance.getWallType()[  # 回転する前
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 1)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.NO_WALL
                            and mapInstance.getWallType()[  # 回転した後
                                mazeEnums.absDirection(
                                    (
                                        mapInstance.frontDirection.value
                                        + turnDir * (_c + 2)
                                        + (90 if s == deviceEnums.Side.LEFT else 270)
                                    )
                                    % 360
                                )
                            ]
                            != mazeEnums.wallType.NO_WALL
                        ):
                            watchVictimFlag[s] = True
                    if (
                        watchVictimFlag[deviceEnums.Side.LEFT]
                        or watchVictimFlag[deviceEnums.Side.RIGHT]
                    ):  # 見る必要がある
                        turnToCertainDirection(  # 90度で止まる
                            (mapInstance.frontDirection.value + turnDir * (_c + 1))
                            % 360,
                            stmInstance,
                            rescueVictim=False,
                            mapInstance=mapInstance,
                            detectedVictimDuringMove=detectedVictimDuringMove,
                        )
                        continue
                    else:
                        # 180度で止まる
                        turnToCertainDirection(
                            (mapInstance.frontDirection.value + turnDir * (_c + 2))
                            % 360,
                            stmInstance,
                            rescueVictim=False,
                            mapInstance=mapInstance,
                            detectedVictimDuringMove=detectedVictimDuringMove,
                        )
                        return
                else:
                    turnToCertainDirection(
                        (mapInstance.frontDirection.value + turnDir * (_c + 1)) % 360,
                        stmInstance,
                        rescueVictim=False,
                        mapInstance=mapInstance,
                        detectedVictimDuringMove=detectedVictimDuringMove,
                    )
                continue

            # ゆっくり90度回る
            turnToCertainDirection(
                (mapInstance.frontDirection.value + turnDir * (_c + 1)) % 360,
                stmInstance,
                rescueVictim=True,
                mapInstance=mapInstance,
                normalTurnSpeed=mazeConstraints.SLOW_DOWN_FOR_VICTIM_DETECTION_WHEN_TURNING_SPEED,
                detectedVictimDuringMove=detectedVictimDuringMove,
            )


def moveTile(
    direction: mazeEnums.absDirection,
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    lidar: ydlidar.CYdLidar,
) -> tuple[bool, bool, bool]:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    @return: (isBlackTile, stoppedByToggleSwitch, resetMapData)
    """

    timing_start = debugTimingPrint(
        f"moveTile start direction={direction} position={mapInstance.currentPosition} front={mapInstance.frontDirection}"
    )

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True, False
    timing_start = debugTimingPrint("moveTile after initial toggle check", timing_start)

    isRedTile = detectTileColor() == mazeEnums.tileType.RED
    isUnknownTileAhead = (
        mapInstance.getTileType(direction) == mazeEnums.tileType.UNKNOWN
    )  # 移動先のタイルがUNKNOWNかどうか。

    timing_start = debugTimingPrint(
        f"moveTile after tile checks isRedTile={isRedTile} isUnknownTileAhead={isUnknownTileAhead}",
        timing_start,
    )

    debugPrint(
        f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}"
    )

    ###### もし旋回するならmapInstance.movedVerticalDistanceをリセット（直進時しか階段判定を行う必要はない） ######
    if direction != mapInstance.frontDirection:
        mapInstance.resetMovedVerticalDistance()

    detectedVictimDuringMove: set[deviceEnums.UnitVStatus] = (
        set()
    )  # 移動中に検出した被災者の種類を記録するセット

    ###### 移動方向へ旋回 ######
    timing_start = debugTimingPrint(
        f"moveTile start heading change target={direction.value}", timing_start
    )
    if (
        mazeConstraints.USE_45_TURN_WITH_VICTIM_CHECK
        and direction != mapInstance.frontDirection
    ):
        turnWith45VictimCheck(
            direction, stmInstance, mapInstance, detectedVictimDuringMove
        )
    elif (
        mazeConstraints.USE_SLOW_DOWN_FOR_VICTIM_DETECTION_WHEN_TURNING
        and direction != mapInstance.frontDirection
    ):
        turnWithSlowVictimCheck(
            direction, stmInstance, mapInstance, detectedVictimDuringMove
        )
    else:
        turnToCertainDirection(
            direction.value,
            stmInstance,
            rescueVictim=True,
            mapInstance=mapInstance,
            detectedVictimDuringMove=detectedVictimDuringMove,
        )

    timing_start = debugTimingPrint("moveTile finished heading change", timing_start)

    ###### 坂道後、PITCHが傾いていたら調整
    if mazeConstraints.ADJUSTMENT_AFTER_RAMP_IF_TILTED_IN_PITCH:
        if (
            min(stmInstance.gyro.getValue().pitch, 360-stmInstance.gyro.getValue().pitch)
            > mazeConstraints.ADJUSTMENT_AFTER_RAMP_IF_TILTED_IN_PITCH_THRESHOLD_DEG
        ):
            if mapInstance.getIsLastMovementOnRamp() != 0:
                debugPrint(
                    "Adjusting after ramp: moving forward slightly due to pitch tilt"
                )
                turnToCertainDirection(
                    mapInstance.frontDirection.value,
                    stmInstance,
                    rescueVictim=False,
                    mapInstance=mapInstance,
                    normalTurnSpeed=mazeConstraints.ADJUSTMENT_AFTER_RAMP_IF_TILTED_IN_PITCH_TURN_SPEED,
                    detectedVictimDuringMove=detectedVictimDuringMove,
                )
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
                time.sleep(
                    mazeConstraints.ADJUSTMENT_AFTER_UP_RAMP_IF_TILTED_IN_PITCH_FORWARD_TIME_SEC
                    if mapInstance.getIsLastMovementOnRamp() == 1
                    else mazeConstraints.ADJUSTMENT_AFTER_DOWN_RAMP_IF_TILTED_IN_PITCH_FORWARD_TIME_SEC
                )
                stmInstance.sts3032.stop()
                turnToCertainDirection(
                    direction.value,
                    stmInstance,
                    rescueVictim=False,
                    mapInstance=mapInstance,
                    normalTurnSpeed=mazeConstraints.ADJUSTMENT_AFTER_RAMP_IF_TILTED_IN_PITCH_TURN_SPEED,
                    detectedVictimDuringMove=detectedVictimDuringMove,
                )
                
    timing_start = debugTimingPrint("Adjustment after ramp completed", timing_start)

    if mazeConstraints.SEE_VICTIM_AFTER_TURNING_AT_NOT_CORNER_But_WALL_IS_PRESENT:
        # 回転前正面方向に壁があって、90°回転であって、90°回転後の後ろに壁がない場合、開店後に少し下がって被災者を確認する
        if mapInstance.getWallType()[mapInstance.frontDirection] != mazeEnums.wallType.NO_WALL and mapInstance.getWallType()[mapInstance.frontDirection] != mazeEnums.wallType.OBSTACLE_WALL:
            if (direction.value - mapInstance.frontDirection.value + 360) % 360 in (90, 270):
                if mapInstance.getWallType()[mazeEnums.absDirection((direction.value + 180) % 360)] == mazeEnums.wallType.NO_WALL or mapInstance.getWallType()[mazeEnums.absDirection((direction.value + 180) % 360)] == mazeEnums.wallType.OBSTACLE_WALL:
                    debugPrint("After turning, checking for victims due to wall presence.")
                    _victim_check_start_time = time.time()
                    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_BACKWARD_LOW_SPEED)
                    s = deviceEnums.Side.RIGHT if (direction.value - mapInstance.frontDirection.value + 360) % 360 == 90 else deviceEnums.Side.LEFT
                    while time.time() - _victim_check_start_time < mazeConstraints.SEE_VICTIM_AFTER_TURNING_AT_NOT_CORNER_But_WALL_IS_PRESENT_BACKWARD_TIME_SEC*2:
                        stmInstance.update()
                        victimInfo = getVictimInfo(stmInstance)
                        if (
                            victimInfo[s] != deviceEnums.UnitVStatus.NOTHING
                            and not mapInstance.isSeenVictimType(
                                [
                                    mazeEnums.absDirection(
                                        (
                                            mapInstance.frontDirection.value
                                            + (90 if s == deviceEnums.Side.LEFT else 270)
                                        )
                                        % 360
                                    )
                                ],
                                victimInfo[s],
                            )
                        ):
                            _before_drop_rescue_kit_time = time.time()
                            stmInstance.sts3032.stop()
                            logger.info(
                                f"Detected victim info after turning at wall: {victimInfo}"
                            )
                            dropRescueKit(
                                stmInstance,
                                mapInstance,
                                victimInfo,
                                s,
                                detectedVictimDuringMove,
                            )
                            mapInstance.addSeenVictimType(
                                [
                                    mazeEnums.absDirection(
                                        (
                                            mapInstance.frontDirection.value
                                            + (90 if s == deviceEnums.Side.LEFT else 270)
                                        )
                                        % 360
                                    )
                                ],
                                victimInfo[s],
                            )
                            _victim_check_start_time += time.time() - _before_drop_rescue_kit_time
                            
                        if time.time() - _victim_check_start_time > mazeConstraints.SEE_VICTIM_AFTER_TURNING_AT_NOT_CORNER_But_WALL_IS_PRESENT_BACKWARD_TIME_SEC:
                            stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
                        else:
                            stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_BACKWARD_LOW_SPEED)
                    stmInstance.sts3032.stop()
                    
    mapInstance.updateFrontDirection(direction)
    timing_start = debugTimingPrint("moveTile updated front direction", timing_start)

    stmInstance.sts3032.stop()
    timing_start = debugTimingPrint(
        "moveTile stopped motors before alignment", timing_start
    )

    # ディスプレイを更新
    stmInstance.rearSTM.update_oled(
        *mapInstance.currentPosition, mapInstance.frontDirection.value
    )

    ##### 移動前の静止時にLiDARの点群を取得。
    pts = LiDAR.getLiDARScan(lidar)

    # DangerousZone内、未探索タイルへの移動で、坂道を検出したら壁と判断。
    if (
        isUnknownTileAhead
        and mapInstance.isStartedDangerousZone
        and mazeConstraints.AVOID_SLOPE_IN_DANGEROUS_ZONE
    ):
        # 坂道判定
        stmInstance.frontTSD10.update()
        if (
            stmInstance.frontTSD10.get_distance() < 300
            and (
                LiDAR.getCertainAngleDist(direction.value, pts)
                - stmInstance.frontTSD10.get_distance() / 10
            )
            > mazeConstraints.WALL_DETECTION_RAMP_THRESHOLD_DIFF_CM
        ):
            mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
            return False, False, False

    ##### 回転後、正面に障害物がないか確認
    detectWall(lidar, mapInstance, stmInstance, pts)
    if (
        mapInstance.getWallType()[mapInstance.frontDirection]
        != mazeEnums.wallType.NO_WALL
    ):
        return False, False, False

    ###### 移動前、真後ろが壁であれば位置調整 ######
    _distance_to_wall = mapInstance.getDistanceToWall()
    _position_adjustment_loop_list = [0, 1]
    if _distance_to_wall[0] is not None and _distance_to_wall[1] is not None:
        if _distance_to_wall[0] < _distance_to_wall[1]:
            _position_adjustment_loop_list = [0, 1]
        else:
            _position_adjustment_loop_list = [1, 0]
    for i in _position_adjustment_loop_list:
        if _distance_to_wall[i] is None:
            continue
        if _distance_to_wall[i] <= mazeConstraints.POSITION_ADJUSTMENT_USING_LIDAR_THRESHOLD_TILE_COUNT:  # type: ignore
            timing_start = debugTimingPrint(
                f"moveTile start position adjustment using LiDAR for {'front' if i==0 else 'back'} wall.",
                timing_start,
            )
            behind_wall_dist = LiDAR.getCertainAngleDist(0 if i == 0 else 180, pts)
            target_behind_dist = (mazeConstraints.POSITION_ADJUSTMENT_USING_LIDAR_FRONT_DISTANCE_CM if i == 0 else mazeConstraints.POSITION_ADJUSTMENT_USING_LIDAR_BACK_DISTANCE_CM) + (  # type: ignore
                mazeConstraints.TILE_SIZE_CM * _distance_to_wall[i]
            )
            # 差が大きすぎたら、怖いので調整しない
            if abs(behind_wall_dist - target_behind_dist) > mazeConstraints.POSITION_ADJUSTMENT_USING_LIDAR_MAX_ADJUSTMENT_CM:  # type: ignore
                debugPrint(
                    f"Position adjustment skipped due to large distance difference: behind_wall_dist={behind_wall_dist}, target_behind_dist={target_behind_dist}"
                )
                continue
            stmInstance.sts3032.setMotorSpeed(
                mazeConstraints.GO_STRAIGHT_LOW_SPEED
                if (behind_wall_dist < target_behind_dist and i == 1)
                or (behind_wall_dist > target_behind_dist and i == 0)
                else mazeConstraints.GO_BACKWARD_LOW_SPEED
            )
            debugPrint("Little backward to adjust position")
            while True:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return False, True, False
                if min(stmInstance.gyro.getValue().roll, 360-stmInstance.gyro.getValue().roll) > 15:
                    break
                scanPoints = LiDAR.getLiDARScan(lidar)
                heading = stmInstance.gyro.getValue().heading
                currentDist = LiDAR.getCertainAngleDist(
                    0 if i == 0 else 180, scanPoints
                )
                if (currentDist > target_behind_dist) == (
                    behind_wall_dist < target_behind_dist
                ):
                    break
            stmInstance.sts3032.stop()
            timing_start = debugTimingPrint(
                "moveTile finished backward adjustment", timing_start
            )
            pts = LiDAR.getLiDARScan(lidar)  # 再度点群を取得しておく
            break

    timing_start = debugTimingPrint("moveTile ready for forward move", timing_start)

    #### 左右の障害物対策

    _judge_front_wall_result = LiDAR.judgeWallCertainAngle(0, pts)
    steer_gain_correction_due_to_obstacle: float = 1.0
    time_correction_due_to_obstacle: float = 0.0
    if _judge_front_wall_result == deviceEnums.judgeWallResult.LEFT_OBSTACLE:
        turnToCertainDirection(
            (mapInstance.frontDirection.value - 25) % 360, stmInstance
        )
        steer_gain_correction_due_to_obstacle = 0.7
        time_correction_due_to_obstacle = 0.1
    elif _judge_front_wall_result == deviceEnums.judgeWallResult.RIGHT_OBSTACLE:
        turnToCertainDirection(
            (mapInstance.frontDirection.value + 25) % 360, stmInstance
        )
        steer_gain_correction_due_to_obstacle = 0.7
        time_correction_due_to_obstacle = 0.1

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True, False

    # dist0, dist180 = LiDAR.getCertainAngleDist([0, 180], pts)
    # nearestLiDARAngle = 0 if dist0 < dist180 else 180
    heading = stmInstance.gyro.getValue().heading
    # oldDist = LiDAR.getCertainAngleDist(
    #     nearestLiDARAngle - heading + direction.value, pts
    # )
    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    isBigRamp = False
    startTime = time.time()
    practicalMoveTime = 0.0
    targetSteps = (
        1  # 現在のマスから何マス先の位置まで移動するか。基本は1。坂道では伸ばす。
    )
    if not isUnknownTileAhead:
        _dist_to_next_tile = mapInstance.getDistanceToNextTile(direction)
        if _dist_to_next_tile is not None:
            targetSteps = round(
                _dist_to_next_tile / mazeConstraints.TILE_SIZE_CM
            )  # 目標とするステップ数を計算
            logger.debug(
                f"Calculated target steps to next tile: {targetSteps} based on distance {_dist_to_next_tile} cm"
            )
    timing_start = debugTimingPrint(
        f"moveTile initialized movement targetSteps={targetSteps}", timing_start
    )

    RollonRamp = []
    RampFinishTime = 0
    practicalVerticalMoveTime = 0.0  # practicalMoveTimeにtanθをかけた値。鉛直方向の移動距離を見積もるために使用。

    oldTime = startTime
    getVictimDict: dict[deviceEnums.Side, defaultdict[deviceEnums.UnitVStatus, int]] = {
        deviceEnums.Side.LEFT: defaultdict(int),
        deviceEnums.Side.RIGHT: defaultdict(int),
    }
    getVictimDict_AfterRamp: dict[
        deviceEnums.Side, defaultdict[deviceEnums.UnitVStatus, int]
    ] = {
        deviceEnums.Side.LEFT: defaultdict(int),
        deviceEnums.Side.RIGHT: defaultdict(int),
    }
    getTileColorDict: defaultdict[mazeEnums.tileType, int] = defaultdict(int)
    cameraBlackTileDetected = False
    isWallAhead: dict[deviceEnums.Side, bool] = {
        s: LiDAR.isWallAheadTile(pts, s)
        for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }

    consequentSearchRes: dict[deviceEnums.Side, deviceEnums.UnitVStatus | None] = {
        s: None for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }
    
    enableStopByDistance = LiDAR.getCertainAngleDist(0, pts) < 60  # 前方の距離が60cm未満なら、距離で停止判定を行えます

    # beforeDist = oldDist
    while True:
        loop_start_time = time.time()
        timing_start = debugTimingPrint("moveTile loop start", timing_start)
        stmInstance.update()
        isBlackTileByCam = camera.detectTileColor() == "BLACK"
        cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return False, True, False

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
        steer *= steer_gain_correction_due_to_obstacle
        leftSpeed = int(max(min(100, baseLeft + steer), -100))
        rightSpeed = int(max(min(100, baseRight - steer), -100))
        timing_start = debugTimingPrint(
            f"moveTile calculated motor speeds left={leftSpeed} right={rightSpeed} gyroSteer={gyroSteer} wallSteer={wallSteer}",
            timing_start,
        )
        if mazeConstraints.SLOW_DOWN_ON_RAMP_IN_DANGEROUS_ZONE:
            if (
                mapInstance.isDangerousTile()
                and min(roll, 360 - roll) > mazeConstraints.RAMP_DEG_THRESHOLD
            ):
                leftSpeed = int(
                    leftSpeed
                    * mazeConstraints.SLOW_DOWN_ON_RAMP_IN_DANGEROUS_ZONE_RATIO
                )
                rightSpeed = int(
                    rightSpeed
                    * mazeConstraints.SLOW_DOWN_ON_RAMP_IN_DANGEROUS_ZONE_RATIO
                )
                timing_start = debugTimingPrint(
                    f"moveTile applied slow down on ramp in dangerous zone left={leftSpeed} right={rightSpeed}",
                    timing_start,
                )
        stmInstance.sts3032.setMotorSpeed(
            {deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed}
        )
        timing_start = debugTimingPrint(
            f"moveTile applied motor speed left={leftSpeed} right={rightSpeed}",
            timing_start,
        )


        if mazeConstraints.USE_ADJUSTMENT_AFTER_RAMP:
            if (
                targetSteps > 1
                and min(roll, 360 - roll)
                < mazeConstraints.ADJUSTMENT_AFTER_RAMP_ROLL_THRESHOLD
                and RampFinishTime == 0
            ):
                stmInstance.sts3032.stop()
                time.sleep(0.5)
                RampFinishTime = time.time()
                print(f"Ramp finished, RampFinishTile : {RampFinishTime }")

        ###### 黒タイル回避処理 ######
        escapeFromBlackTileRes = escapeFromBlackTile(
            stmInstance,
            mapInstance,
            direction,
            practicalMoveTime,
            cameraBlackTileDetected,
            getVictimDict,
        )
        if escapeFromBlackTileRes:
            debugTimingPrint("moveTile escaped from black tile", timing_start)
            if mazeConstraints.SEE_VICTIM_WHEN_TURNING_BACKWARD_FROM_BLACK_TILE:
                maxVictimInfo = {
                    side: (
                        max(getVictimDict[side], key=lambda k: getVictimDict[side][k])
                        if getVictimDict[side]
                        else deviceEnums.UnitVStatus.NOTHING
                    )
                    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
                }
                for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                    if (  # 見たことがない、被災者を発見した、壁がある　ならばレスキューキットを落とす
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

                        logger.info(f"Decided victim on {side} side: {maxVictimInfo[side]} during black tile escape")
                        
                        isUpRamp = targetSteps > 1 and RollonRamp[-1] < 180

                        dropRescueKit(
                            stmInstance, mapInstance, maxVictimInfo, side, detectedVictimDuringMove
                        )

                        # SeenVictimTypeに追加。
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

            return True, False, False
        else:
            tempTileColor = detectTileColor()
            if tempTileColor != mazeEnums.tileType.EMPTY:
                getTileColorDict[tempTileColor] += 1
            if tempTileColor == mazeEnums.tileType.RED:
                isRedTile = True
        timing_start = debugTimingPrint(
            f"moveTile tile color check tempTileColor={tempTileColor}", timing_start
        )

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
                practicalMoveTime -= 0.11
                escapeFlag = True
                time.sleep(0.1)
        timing_start = debugTimingPrint(
            f"moveTile obstacle check escapeFlag={escapeFlag}", timing_start
        )
        stmInstance.update()
        
        ##### 坂道判定 #####
        roll = stmInstance.gyro.getValue().roll
        if min(roll, 360 - roll) > mazeConstraints.RAMP_DEG_THRESHOLD:
            isBigRamp = True
        else:
            isBigRamp = False
        timing_start = debugTimingPrint(
            f"moveTile ramp check isBigRamp={isBigRamp}", timing_start
        )

        ###### 1マス移動完了判定（時間制御） ######
        if (
            practicalMoveTime
            > mazeConstraints.MOVE_STRAIGHT_SEC * targetSteps
            + time_correction_due_to_obstacle
        ):
            if isUnknownTileAhead and isBigRamp:
                isContinueRampFlag = True
                if mazeConstraints.USE_JUDGE_AS_END_OF_RAMP_IF_ROLL_DIFF and targetSteps > 1:
                    # roll角が変化してたら坂道の終わりと判断
                    if RollonRamp[-1] < 180 and roll < 180:
                        if (RollonRamp[-1] > roll) and (RollonRamp[-1] - roll) > mazeConstraints.JUDGE_AS_END_OF_RAMP_IF_ROLL_DIFF_DEG:
                            isContinueRampFlag = False
                    elif RollonRamp[-1] > 180 and roll > 180:
                        if (RollonRamp[-1] < roll) and (roll - RollonRamp[-1]) > mazeConstraints.JUDGE_AS_END_OF_RAMP_IF_ROLL_DIFF_DEG:
                            isContinueRampFlag = False
                if isContinueRampFlag:
                    targetSteps += 1
                    RollonRamp.append(roll)
                    getVictimDict_AfterRamp = {
                        deviceEnums.Side.LEFT: defaultdict(int),
                        deviceEnums.Side.RIGHT: defaultdict(int),
                    }
                    continue
                else:
                    logger.info(f"Ramp ended due to roll difference. last RollonRamp: {RollonRamp[-1]}, Current roll: {roll}")
                    # まだ少し坂道にかぶってるってことだから少し直進しよう
                    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
                    time.sleep(mazeConstraints.JUDGE_AS_END_OF_RAMP_IF_ROLL_DIFF_LITTLE_FORWARD_TIME_SEC)
                    stmInstance.sts3032.stop()
            logger.info(
                f"practicalVerticalMove: {practicalVerticalMoveTime*mazeConstraints.TILE_SIZE_CM/mazeConstraints.MOVE_STRAIGHT_SEC} CM"
            )
            if (
                targetSteps > 1
                and mazeConstraints.USE_ADJUSTMENT_AFTER_RAMP
                and (
                    (time.time() - RampFinishTime)
                    < mazeConstraints.ADJUSTMENT_AFTER_RAMP_TIME_SEC
                )
            ):
                logger.info("Waiting for adjustment after ramp")
                while True:
                    stmInstance.update()
                    if stmInstance.switch.getToggleSwitch1():
                        stmInstance.sts3032.stop()
                        return False, True, False
                    if (
                        time.time() - RampFinishTime
                        > mazeConstraints.ADJUSTMENT_AFTER_RAMP_TIME_SEC
                    ):
                        break
            stmInstance.sts3032.stop()
            debugTimingPrint("moveTile finished by time control", timing_start)
            break
    
        if enableStopByDistance and (
            stmInstance.tof.getDistance()[0]
            < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM
            and not isBigRamp
        ):
            stmInstance.frontTSD10.update()
            if (
                stmInstance.tof.getDistance()[0]
                - stmInstance.frontTSD10.get_distance() / 10
            ) < 10:
                # 前方に坂がないときで
                # 前方の壁までの距離が小さくなったときは止める。
                stmInstance.sts3032.stop()
                debugTimingPrint(
                    f"moveTile finished by front distance control. Tof: {stmInstance.tof.getDistance()[0]}, TSD10 : {stmInstance.frontTSD10.get_distance() / 10}", timing_start
                )
                break

        ### 坂道で前方の壁を検知したら、引き返す処理
        if mazeConstraints.TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_RAMP and isBigRamp:
            detectFlag = False
            if (
                practicalMoveTime
                > mazeConstraints.TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_RAMP_ENABLE_TIME_SEC
            ):
                if stmInstance.gyro.getValue().roll < 180:  # 上り坂
                    if (
                        stmInstance.tof.getDistance()[0]
                        < mazeConstraints.TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_UP_RAMP_THRESHOLD_CM
                    ):
                        stmInstance.sts3032.stop()
                        logger.info(f"Front wall detected on up ramp, stopping movement. Tof: {stmInstance.tof.getDistance()[0]}")
                        detectFlag = True
                        practicalMoveTime -= 0.14
                else:
                    if (
                        stmInstance.tof.getDistance()[0]
                        < mazeConstraints.TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_DOWN_RAMP_THRESHOLD_CM
                    ):
                        stmInstance.sts3032.stop()
                        logger.info(f"Front wall detected on down ramp, stopping movement. Tof: {stmInstance.tof.getDistance()[0]}")
                        detectFlag = True
                        practicalMoveTime += 0.4  # 補正
            if detectFlag:
                # 下がる
                _practicalMoveTime = 0
                _startTime = time.time()
                while (
                    _practicalMoveTime < practicalMoveTime
                ):  # TODO: 直進に壁の距離とジャイロの補正をかけるようにする？
                    stmInstance.sts3032.setMotorSpeed(
                        mazeConstraints.GO_BACKWARD_MAX_SPEED
                    )
                    stmInstance.update()
                    if stmInstance.switch.getToggleSwitch1():
                        stmInstance.sts3032.stop()
                        return False, True, False
                    roll = stmInstance.gyro.getValue().roll
                    _correction_factor = (
                        (0.80 if (roll > 6 and roll < 180) else 1)
                        if not (roll > 180 and roll < 350)
                        else 1.1
                    )
                    _practicalMoveTime += (
                        (time.time() - _startTime)
                        * np.cos(np.radians(abs(roll)))
                        * _correction_factor
                    )
                    _startTime = time.time()
                stmInstance.sts3032.stop()
                mapInstance.setTileType(
                    mazeEnums.tileType.BLACK, direction
                )  # 黒タイルとして登録
                return True, False, False

        ####### 被災者発見処理 ######
        victimRescueFlag = False
        if (
            min(roll, 360 - roll) < mazeConstraints.RAMP_DEG_THRESHOLD
        ):  # 坂道でない場合のみ被災者検知を行う
            _useBackwardMove = not (
                targetSteps > 1 and roll < 180
            )  # 坂道で上り坂の場合は後退移動を使用しない
            victimRescueFlag = findVictimDuringMove(
                mapInstance,
                stmInstance,
                isWallAhead,
                consequentSearchRes,
                getVictimDict,
                practicalMoveTime,
                detectedVictimDuringMove,
                useBackwardMove=_useBackwardMove,
            )
        else:
            ######## 坂道上の被災者検知 ######
            # getVictimDict_AfterRampに被災者情報を格納する。救助は坂を上ってor下ってから行う。
            for s in deviceEnums.Side:
                unitvStatus = stmInstance.unitv.getStatus()[s]
                if unitvStatus != deviceEnums.UnitVStatus.NOTHING:
                    getVictimDict_AfterRamp[s][unitvStatus] += 1
                    logger.debug(
                        f"Detected victim info during ramp movement: {unitvStatus} on {s.name} side"
                    )

        if not victimRescueFlag:
            pratical_loop_time = 0
            if escapeFlag:
                pratical_loop_time = timeBeforeEscape - oldTime
            else:
                pratical_loop_time = time.time() - oldTime

            correction_factor = (
                (0.80 if (roll > 6 and roll < 180) else 1)
                if not (roll > 180 and roll < 350)
                else 1.1
            )
            if mazeConstraints.SLOW_DOWN_ON_RAMP_IN_DANGEROUS_ZONE:
                if (
                    mapInstance.isDangerousTile()
                    and min(roll, 360 - roll) > mazeConstraints.RAMP_DEG_THRESHOLD
                ):
                    correction_factor *= (
                        mazeConstraints.SLOW_DOWN_ON_RAMP_IN_DANGEROUS_ZONE_RATIO
                    )

            practicalMoveTime += (
                pratical_loop_time * np.cos(np.radians(abs(roll))) * correction_factor
            )
            practicalVerticalMoveTime += (
                pratical_loop_time * math.sin(math.radians(roll)) * correction_factor
            )

        debugPrint(
            f"roll: {stmInstance.gyro.getValue().roll} deg, practicalMoveTime: {practicalMoveTime} sec"
        )
        oldTime = time.time()
        logger.debug(
            f"moveTile loop time: {(time.time() - loop_start_time) * 1000:.1f} ms"
        )
        timing_start = debugTimingPrint(
            f"moveTile loop end practicalMoveTime={practicalMoveTime:.3f}", timing_start
        )

    ###### 移動後、目の前が壁であれば位置調整のため少し前進 ######
    timing_start = debugTimingPrint(
        "moveTile start final forward adjustment", timing_start
    )

    pts = LiDAR.getLiDARScan(lidar)
    if LiDAR.judgeWallCertainAngle(0, pts) == deviceEnums.judgeWallResult.WALL:
    # if 15 < LiDAR.getCertainAngleDist(-heading + direction.value, pts) < 27:
        stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")
        _startTime = time.time()
        while True:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return False, True, False
            if min(stmInstance.gyro.getValue().roll, 360-stmInstance.gyro.getValue().roll) > 10:
                stmInstance.sts3032.stop()
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_BACKWARD_LOW_SPEED)
                time.sleep(0.16)
                stmInstance.sts3032.stop()
                break
            scanPoints = LiDAR.getLiDARScan(lidar)
            last_scan_points = scanPoints
            heading = stmInstance.gyro.getValue().heading
            currentDist = LiDAR.getCertainAngleDist(
                -heading + direction.value, scanPoints
            )
            currentDist = stmInstance.tof.getDistance()[0]
            logger.info(f"Final forward adjustment... Current dist:{currentDist}")
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
            if currentDist > 30:
                break
            if (
                time.time() - _startTime > 0.8
            ):  # 0.8秒以上経っても距離が縮まらない場合は坂か階段か何かだったと判断し、引き返す
                backwardTime = time.time() - _startTime
                stmInstance.sts3032.stop()
                logger.warning("Final forward adjustment timeout, stopping adjustment")
                
                stmInstance.update()
                if 10 < stmInstance.gyro.getValue().roll < 180:  # 上り坂
                    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_BACKWARD_LOW_SPEED)
                    time.sleep(backwardTime)
                    stmInstance.sts3032.stop()
                break


    stmInstance.sts3032.stop()
    timing_start = debugTimingPrint(
        "moveTile finished final forward adjustment", timing_start
    )

    ##### 坂を上ったのであればマップに登録 #####
    if targetSteps > 1 and isUnknownTileAhead:
        if targetSteps == 2:
            pastMovedVerticalDistance = mapInstance.getMovedVerticalDistance()
            # 登り坂では坂検知をしなかったが、下り坂で坂検知をした時の例外処理
            if (
                RollonRamp[0] > 180
                and abs(
                    sum(d for d, _ in pastMovedVerticalDistance)
                    + practicalVerticalMoveTime
                    * mazeConstraints.TILE_SIZE_CM
                    / mazeConstraints.MOVE_STRAIGHT_SEC
                )
                < mazeConstraints.STAIR_THRESHOLD_CM
                and (not any(f for _, f in pastMovedVerticalDistance))
            ):
                logger.info(
                    "Detected ramp but regarded it as down stairs, treating as normal tile"
                )
                mapInstance.moveTo(direction)
                # 左右に壁を設定
                for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                    mapInstance.setWallType(
                        mazeEnums.absDirection(
                            (
                                direction.value
                                + (90 if s == deviceEnums.Side.LEFT else 270)
                            )
                            % 360
                        ),
                        mazeEnums.wallType.WALL,
                    )
                    # 前後にはNO_WALLを設定
                    mapInstance.setWallType(
                        mazeEnums.absDirection(
                            (
                                direction.value
                                + (0 if s == deviceEnums.Side.LEFT else 180)
                            )
                            % 360
                        ),
                        mazeEnums.wallType.NO_WALL,
                    )
                    # TileTypeを設定
                    mapInstance.setTileType(mazeEnums.tileType.EMPTY)
                mapInstance.resetMovedVerticalDistance()

            # 1マスの階段を坂と誤検知したときの例外処理
            if (
                abs(
                    practicalVerticalMoveTime
                    * mazeConstraints.TILE_SIZE_CM
                    / mazeConstraints.MOVE_STRAIGHT_SEC
                )
                < mazeConstraints.STAIR_THRESHOLD_CM
            ):
                logger.info(
                    f"Detected one-tile ramp but regarded it as stairs on one tile, set height-0 slope. Vertical Movement: {practicalVerticalMoveTime  * mazeConstraints.TILE_SIZE_CM   / mazeConstraints.MOVE_STRAIGHT_SEC} cm"
                )
                mapInstance.setSlope(direction, mazeConstraints.TILE_SIZE_CM, 0)
                mapInstance.setMovedVerticalDistance(
                    practicalVerticalMoveTime
                    * mazeConstraints.TILE_SIZE_CM
                    / mazeConstraints.MOVE_STRAIGHT_SEC,
                    False,
                )
            else:
                mapInstance.setSlope(
                    direction,
                    (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM,
                    (targetSteps - 1)
                    * 15
                    * (
                        1 if RollonRamp[-1] < 180 else -1
                    ),  # 1マスなら15cmの高さ差があると仮定する。
                )
                logger.info(
                    f"Set slope for tile at {direction}, horizontal: {(targetSteps - 1) * mazeConstraints.TILE_SIZE_CM} cm, vertical: {(targetSteps - 1) * 15 * (1 if RollonRamp[-1] < 180 else -1)} cm based on roll {RollonRamp[-1]} deg"
                )
                if RollonRamp[-1] < 180:
                    mapInstance.setMovedVerticalDistance(
                        practicalVerticalMoveTime
                        * mazeConstraints.TILE_SIZE_CM
                        / mazeConstraints.MOVE_STRAIGHT_SEC,
                        True,
                    )
                else:
                    mapInstance.setMovedVerticalDistance(0, False)
        else:
            if RollonRamp[0] < 180 and RollonRamp[-1] > 180:  # 階段だった
                # 高さ0のスロープを設置
                mapInstance.setSlope(
                    direction,
                    (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM,
                    0,
                )
                logger.info("Detected stairs, set slope with 0cm height difference")

                # for i in range(targetSteps - 1):
                #     mapInstance.moveTo(direction)
                #     # 左右に壁を設定
                #     for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                #         mapInstance.setWallType(
                #             mazeEnums.absDirection(
                #                 (
                #                     direction.value
                #                     + (90 if s == deviceEnums.Side.LEFT else 270)
                #                 )
                #                 % 360
                #             ),
                #             mazeEnums.wallType.WALL,
                #         )
                #         # 前後にはNO_WALLを設定
                #         mapInstance.setWallType(
                #             mazeEnums.absDirection(
                #                 (
                #                     direction.value
                #                     + (0 if s == deviceEnums.Side.LEFT else 180)
                #                 )
                #                 % 360
                #             ),
                #             mazeEnums.wallType.NO_WALL,
                #         )
                #         # TileTypeを設定
                #         mapInstance.setTileType(
                #             mazeEnums.tileType.EMPTY
                #         )
                mapInstance.resetMovedVerticalDistance()
            else:
                mapInstance.setSlope(
                    direction,
                    (targetSteps - 1) * mazeConstraints.TILE_SIZE_CM,
                    (targetSteps - 1)
                    * mazeConstraints.TILE_SIZE_CM
                    * math.tan(math.radians(RollonRamp[-1])),
                )
                logger.info(
                    f"Set slope for tile at {direction}, horizontal: {(targetSteps - 1) * mazeConstraints.TILE_SIZE_CM}, height {(targetSteps - 1) * mazeConstraints.TILE_SIZE_CM * math.tan(math.radians(RollonRamp[-1]))} cm based on roll {RollonRamp[-1]} deg"
                )
                mapInstance.resetMovedVerticalDistance()

    #### movedVerticalDistanceを更新、上り坂検出下り坂未検出の階段検知
    if targetSteps == 1 and isUnknownTileAhead:
        pastMovedVerticalDistance = mapInstance.getMovedVerticalDistance()
        thisMovedVerticalDistance = (
            practicalVerticalMoveTime
            * mazeConstraints.TILE_SIZE_CM
            / mazeConstraints.MOVE_STRAIGHT_SEC
        )
        if (
            sum(d for d, _ in pastMovedVerticalDistance) + thisMovedVerticalDistance
            < mazeConstraints.STAIR_THRESHOLD_CM
            and thisMovedVerticalDistance < 0
        ):

            # 上りのときの坂検出していたか確認
            if any(d > 0 and isSlope for d, isSlope in pastMovedVerticalDistance):
                logger.info(
                    "Detected stair after up ramp, set 0cm slope on stair tile."
                )

                mapInstance.setSlope(
                    direction, 0, -15
                )  # 1マスで移動した時しかこのように判断しないはずなので、高さ差15cmの坂。
                mapInstance.resetMovedVerticalDistance()
            # 階段と判定
            pass
        if True:
            logger.info(
                f"Set moved vertical distance: {thisMovedVerticalDistance} cm, past distances: {[(d, isSlope) for d, isSlope in pastMovedVerticalDistance]}"
            )
        mapInstance.setMovedVerticalDistance(thisMovedVerticalDistance, False)
        timing_start = debugTimingPrint(
            f"moveTile updated moved vertical distance {thisMovedVerticalDistance:.3f}",
            timing_start,
        )

    # もし坂判定をして進んだのに坂ではなかったら、壁を置いて、進まなかったことにする。
    if mapInstance.isSlopeDetected and targetSteps == 1:
        logger.info(f"Ramp was detected but there was no ramp, treating as wall. ")
        mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
    # 進んだ距離があまりにも短すぎたら壁を置いて進まなかったことにする
    elif practicalMoveTime < 0.3:
        logger.info(
            f"Moved distance is too small, treating as wall. practicalMoveTime: {practicalMoveTime}"
        )
        mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
    else:
        mapInstance.moveTo(direction)

    stmInstance.rearSTM.update_oled(
        *mapInstance.currentPosition, mapInstance.frontDirection.value
    )

    mapInstance.isSlopeDetected = False

    detectWallRes = detectWall(lidar, mapInstance, stmInstance)
    if (
        mazeConstraints.DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR
        and detectWallRes == False
    ):
        mapInstance.resetMapData()
        detectWall(lidar, mapInstance, stmInstance)  # 壁検出をやり直す
        logger.warning("Wall detection error, resetting internal map data")
        return True, False, True

    debugTimingPrint("moveTile updated map and detected walls", timing_start)

    ##### 移動終了時の被災者検出処理 #####
    ##### 移動終了直前、移動終了時に見たもの、上り坂をのぼったあとの特殊処理で見たものについて、ここで救助動作を行う。
    # 坂道中に見たものを追加
    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        for unitvStatus, count in getVictimDict_AfterRamp[side].items():
            getVictimDict[side][unitvStatus] += count

    maxVictimInfo = {
        side: (
            max(getVictimDict[side], key=lambda k: getVictimDict[side][k])
            if getVictimDict[side]
            else deviceEnums.UnitVStatus.NOTHING
        )
        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]
    }
    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        if (  # 壁ですか？？
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
        if not (
            consequentSearchRes[side] is None
        ):  # 連続壁被災者検出をしていたら、addSeenVictimだけする
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
                consequentSearchRes[side],  # type: ignore
            )
        elif (  # 見たことがない、被災者を発見した、壁がある　ならばレスキューキットを落とす
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

            logger.info(f"Decided victim on {side} side: {maxVictimInfo[side]}")
            
            isUpRamp = targetSteps > 1 and RollonRamp[-1] < 180
            
            stmInstance.update()
            _back_is_wall = stmInstance.tof.getDistance()[3] < 21
            
            if mazeConstraints.BACKWARD_AFTER_DROP_KIT and not isUpRamp and not _back_is_wall:
                logger.info("Moving backward before dropping rescue kit")
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_BACKWARD_LOW_SPEED)
                time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                stmInstance.sts3032.stop()

            dropRescueKit(
                stmInstance, mapInstance, maxVictimInfo, side, detectedVictimDuringMove
            )

            if mazeConstraints.BACKWARD_AFTER_DROP_KIT and not isUpRamp and not _back_is_wall:
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
                time.sleep(mazeConstraints.BACKWARD_AFTER_DROP_KIT_TIME_SEC)
                stmInstance.sts3032.stop()

            # SeenVictimTypeに追加。
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

    ###### 銀・青タイル判別処理 ######
    tileType = mazeEnums.tileType.EMPTY

    ### 過去の検出結果をもとに最大のものを選択する手法
    nowMaxCount = 0
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

    tileType = detectTileColor()
    mapInstance.setTileType(
        tileType if tileType != mazeEnums.tileType.BLACK else mazeEnums.tileType.EMPTY
    )
    logger.info(f"Tile color detection, decided tile type: {tileType}")
    if mapInstance.getTileType() == mazeEnums.tileType.SILVER:
        stmInstance.rearSTM.playMusic(buzzerSongs.checkpoint)
    if mapInstance.getTileType() == mazeEnums.tileType.RED:
        stmInstance.rearSTM.playMusic(buzzerSongs.redTile)

    # if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
    #     stmInstance.rearSTM.playMusic(buzzerSongs.swamp)
    #     time.sleep(5.2)
    if mapInstance.getTileType() != mazeEnums.tileType.EMPTY:
        logger.info(
            f"Moved to {mapInstance.currentPosition}, Tile type: {mapInstance.getTileType()}, Wall types: {mapInstance.getWallType()}"
        )
    debugTimingPrint("moveTile finished", timing_start)
    stmInstance.sts3032.stop()
    return False, False, False


def moveNextTile(
    direction: mazeEnums.absDirection,
    mapInstance: mazeMap.mazeMap,
    stmInstance: stm.STM,
    lidar: ydlidar.CYdLidar,
) -> tuple[bool, bool, bool]:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    @return: isBlackTile, stoppedByToggleSwitch, resetMapData
    """
    isBlack, stopped, resetMapData = moveTile(
        direction, mapInstance, stmInstance, lidar
    )

    if not isBlack:
        stmInstance.sts3032.stop()

    logger.info(mapInstance.getDistanceToWall())

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return isBlack, True, resetMapData
    return isBlack, stopped, resetMapData
