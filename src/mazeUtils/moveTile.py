import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap
from .device import LiDAR, deviceConstraints, deviceEnums, stm, camera,buzzerSongs
import time
import threading
import numpy as np
import math
from collections import defaultdict
colorSensor = None
pr = None

lastDist = {mazeEnums.absDirection.NORTH: 0, mazeEnums.absDirection.EAST: 0, mazeEnums.absDirection.SOUTH: 0, mazeEnums.absDirection.WEST: 0}

def turnOnLED(stmInstance: stm.STM,mapInstance: mazeMap.mazeMap, color: list[int]) -> None:
    """
    @brief LEDを点灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(*color)
    #mapInstance.arduinoNanoEvery.victimled(tuple(color))

def turnOffLED(stmInstance: stm.STM,mapInstance: mazeMap.mazeMap) -> None:
    """
    @brief LEDを消灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(0,0,0)
    #mapInstance.arduinoNanoEvery.victimled((0, 0, 0))

def flashLED(stmInstance: stm.STM, mapInstance: mazeMap.mazeMap, loopCount: int, intervalSec: float, color: list[int]) -> None:
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
        print(*message)

class LiDARScanCache:
    def __init__(self, lidar: ydlidar.CYdLidar, update_interval_sec: float = 0.13) -> None:
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

def isUturn(fromDir: int, toDir: int, mapInstance: mazeMap.mazeMap) -> bool:
    turnAngle = fromDir - toDir
    if turnAngle > 180:
        turnAngle -= 360
    if turnAngle < -180:
        turnAngle += 360
    
    if abs(turnAngle) == 180 and (mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] <= 1 and mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] <= 1):
        return True
    else:
        return False

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

def getQuantizedDir(dir: int) -> list[mazeEnums.absDirection]:
    """
    @brief 方向を最も近い2方向に量子化する
    @param dir: 量子化する方向 (0-359)
    @return: 量子化された方向のタプル
    """
    dir = dir % 360
    directions = [mazeEnums.absDirection.NORTH, mazeEnums.absDirection.EAST, mazeEnums.absDirection.SOUTH, mazeEnums.absDirection.WEST]
    diffs = [abs(regulationAngle(dir - d.value)) for d in directions]
    sortedIndices = np.argsort(diffs)
    return [directions[sortedIndices[0]], directions[sortedIndices[1]]]

def turnToCertainDirection(targetDir: int, stmInstance: stm.STM, rescueVictim: bool = False, mapInstance: mazeMap.mazeMap = None) -> None:
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

        stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD)
        debugPrint("current heading:", stmInstance.gyro.getValue().heading, "target:", targetDir)
        oldTurnDir = turnDirection
        rescueStopped = False
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            if rescueVictim:
                for s in deviceEnums.Side:
                    victimInfo = getVictimInfo(stmInstance)
                    tiletype = detectTileColor()
                    if victimInfo[s] != deviceEnums.UnitVStatus.NOTHING and mapInstance.isSeenVictimType(getQuantizedDir(stmInstance.gyro.getValue().heading + (90 if s == deviceEnums.Side.LEFT else 270)), victimInfo[s]) == False and mapInstance.getWallType()[getQuantizedDir(stmInstance.gyro.getValue().heading + (90 if s == deviceEnums.Side.LEFT else 270))[0]] == mazeEnums.wallType.WALL and mapInstance.getWallType()[getQuantizedDir(stmInstance.gyro.getValue().heading + (90 if s == deviceEnums.Side.LEFT else 270))[1]] == mazeEnums.wallType.WALL:
                        if not (tiletype == mazeEnums.tileType.RED and victimInfo[s] == deviceEnums.UnitVStatus.R_VICTIM):
                            stmInstance.sts3032.stop()
                            rescueStopped = True
                            print(f"Find victim on {'LEFT' if s == deviceEnums.Side.LEFT else 'RIGHT'} side during turn: {victimInfo[s]}")
                            mapInstance.addSeenVictimType(getQuantizedDir(stmInstance.gyro.getValue().heading + (90 if s == deviceEnums.Side.LEFT else 270)), victimInfo[s])
                            dropRescueKit(stmInstance, mapInstance, victimInfo, s)
                            print(f"Dropped rescue kit, detected victim info: {victimInfo}")
            turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
            if rescueStopped or turnDirection != oldTurnDir:
                stmInstance.sts3032.turnRight(mazeConstraints.TURN_SPD) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(mazeConstraints.TURN_SPD)
                oldTurnDir = turnDirection
                rescueStopped = False

        assert abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) <= mazeConstraints.TURN_THRESHOLD_DEG, f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
        stmInstance.sts3032.stop()

        print("stopped turning at heading:", stmInstance.gyro.getValue().heading, "diff:" , abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)))
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return
        oldTurnDir = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
        stmInstance.sts3032.turnRight(5) if oldTurnDir == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(5)
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX:
            if oldTurnDir != getTurnDirection(stmInstance.gyro.getValue().heading, targetDir):
                stmInstance.sts3032.turnRight(5) if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir) == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(5)
                oldTurnDir = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
        stmInstance.sts3032.stop()
    else: # PD制御
        turnAngle = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        debugPrint(f"Turning to {targetDir} deg, current heading: {stmInstance.gyro.getValue().heading} deg, turnAngle: {turnAngle} deg")

        oldError = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        oldTime = time.time()
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX:
            if time.time() - oldTime > mazeConstraints.TIMEOUT_FOR_TURNING_SEC:
                debugPrint("Turn timeout reached.")
                t = time.time()
                while time.time() - t < 0.2:
                    stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -50, deviceEnums.Side.RIGHT: -50})
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
            turnSpeed = mazeConstraints.TURN_P * error + mazeConstraints.TURN_D * deribative
            turnSpeed = max(min(turnSpeed, 100), -100)
            turnSpeed = turnSpeed if abs(turnSpeed) >= 1 else (1 if turnSpeed > 0 else -1)
            stmInstance.sts3032.turnRight(abs(int(turnSpeed))) if turnSpeed > 0 else stmInstance.sts3032.turnLeft(abs(int(turnSpeed)))
            debugPrint(f"gyro:{stmInstance.gyro.getValue().heading} deg, derivative: {deribative}, turnSpeed: {turnSpeed}")
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
            #elif direction == mapInstance.frontDirection: and ((LiDAR.getCertainAngleDist(0,points) - (mapInstance.arduinoNanoEvery.request_tof_distance_mm()/10 + 10)) > mazeConstraints.RAMP_TOF_THRESHOLD and LiDAR.getCertainAngleDist(0,points) < mazeConstraints.JUDGE_RAMP_LIDAR_THRESHOLD):
            #    mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
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
    flashLED(stmInstance, mapInstance, 5, 0.5,color=[(0,255,0),(255,255,0),(255,0,0)][needRescueKitCount])
    oppositeFlag = False
    tileColor = detectTileColor()
    firstHeading = stmInstance.gyro.getValue().heading
    if tileColor == mazeEnums.tileType.RED:
        return
    for i in range(needRescueKitCount):
        if mapInstance.nowRescueKitCount[side if not oppositeFlag else side.opposite()] >= 1:
            stmInstance.update()
            turnToCertainDirection((stmInstance.gyro.getValue().heading + (i+1)*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.dropRescueKit(side if not oppositeFlag else side.opposite(), 1)
            stmInstance.rescuekitservo.dropRescueKit(1, side if not oppositeFlag else side.opposite())
            oldtime = time.time()
            while time.time() - oldtime < 0.8:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return
        elif mapInstance.nowRescueKitCount[side.opposite() if not oppositeFlag else side] >= 1:
            turnToCertainDirection((stmInstance.gyro.getValue().heading + 180 + (i+1)*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.updateFrontDirection(
                mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
            )
            mapInstance.dropRescueKit(side.opposite() if not oppositeFlag else side, 1)
            stmInstance.rescuekitservo.dropRescueKit(1, side.opposite() if not oppositeFlag else side)
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
    
def rescueVictim(mapInstance: mazeMap.mazeMap, stmInstance: stm.STM) -> None:
    """
    @brief 被災者を救助する動作を行う
    @param mapInstance: 現在の迷路情報
    @param stmInstance: 通信に使用する STM インスタンス
    """     
    stmInstance.update()
    if mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] <= 1 or mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] <= 1:
        time.sleep(0.1)
        t = time.time()
        leftFlag = True
        rightFlag = True
        while time.time() - t < 0.5:            
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            victimInfo = getVictimInfo(stmInstance)
            tiletype = detectTileColor()
            if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.isSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)], victimInfo[deviceEnums.Side.LEFT]) == False and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL and leftFlag:
                if not (tiletype == mazeEnums.tileType.RED and victimInfo[deviceEnums.Side.LEFT] == deviceEnums.UnitVStatus.R_VICTIM):                    
                    print(f"Find victim on LEFT side: {victimInfo[deviceEnums.Side.LEFT]}")
                    mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)], victimInfo[deviceEnums.Side.LEFT])
                    dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.LEFT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    leftFlag = False

            if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.isSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)], victimInfo[deviceEnums.Side.RIGHT]) == False and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL and rightFlag:
                if not (tiletype == mazeEnums.tileType.RED and victimInfo[deviceEnums.Side.RIGHT] == deviceEnums.UnitVStatus.R_VICTIM):
                    print(f"Find victim on RIGHT side: {victimInfo[deviceEnums.Side.RIGHT]}")  
                    mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)], victimInfo[deviceEnums.Side.RIGHT])
                    dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    rightFlag = False
            stmInstance.update()
        mapInstance.addSeenCount()

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stmInstance: stm.STM, lidar: ydlidar.CYdLidar, firstTime: float) -> tuple[bool,bool]:
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

    isRedTile = detectTileColor() == mazeEnums.tileType.RED

    debugPrint(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")
    turnToCertainDirection(direction.value, stmInstance, rescueVictim=True, mapInstance=mapInstance)
    mapInstance.updateFrontDirection(direction)
    point = LiDAR.getLiDARScan(lidar)
    """
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
    oldDist = LiDAR.getCertainAngleDist(nearestLiDARAngle - heading + direction.value, points)
    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    isRamp = True
    startTime = time.time()
    practicalMoveTime = 0.0
    oldTime = startTime
    getVictimDict = {deviceEnums.Side.LEFT: defaultdict(int), deviceEnums.Side.RIGHT: defaultdict(int)}
    getTileColorDict = defaultdict(int)
    cameraBlackTileDetected = False
    isWallAhead = {s: LiDAR.isWallAheadTile(points, s) for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    dx = mazeEnums.directionToDelta[direction][0]
    dy = mazeEnums.directionToDelta[direction][1]
    currentPosX, currentPosY = mapInstance.currentPosition
    avoidVictim = {s: mapInstance.getSeenVictimType(currentPosX, currentPosY)[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)] | mapInstance.getSeenVictimType(currentPosX + dx, currentPosY + dy)[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if s == deviceEnums.Side.LEFT else 270)) % 360)] for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    print(avoidVictim)
    consequentSearchRes = {s: None for s in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}

    beforeDist = oldDist
    while True:
        loop_start_time = time.time()
        stmInstance.update()
        isBlackTileByCam = False #camera.detectTileColor() == "BLACK"
        cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return False, True
        

        heading = stmInstance.gyro.getValue().heading
        roll = stmInstance.gyro.getValue().roll

        turnAngle = regulationAngle(heading - direction.value)   
        leftWallDist = stmInstance.tof.getDistance()[3]
        rightWallDist = stmInstance.tof.getDistance()[1]
        gyroSteer = turnAngle * mazeConstraints.STRAGIHT_GYRO_P_GAIN
        wallSteer = 0.0

        enableDist = mazeConstraints.WALL_FOLLOW_ENABLE_DIST_CM
        targetDist = mazeConstraints.WALL_FOLLOW_TARGET_DIST_CM
        # ジャイロ誤差を最優先で減らしつつ、壁が近い場合のみ壁距離制御を足す
        if (leftWallDist <= enableDist or rightWallDist <= enableDist) and abs(turnAngle) <= mazeConstraints.WALL_FOLLOW_GYRO_ERR_MAX_DEG:
            if leftWallDist <= enableDist and rightWallDist <= enableDist:
                wallError = rightWallDist - leftWallDist  # 両側が近いなら左右差を0へ
            elif leftWallDist <= enableDist:
                wallError = targetDist - leftWallDist  # 左が近いなら左距離を目標へ
            else:
                wallError = rightWallDist - targetDist  # 右が近いなら右距離を目標へ

            wallSteer = wallError * mazeConstraints.WALL_FOLLOW_P_GAIN
            wallSteer = max(min(wallSteer, mazeConstraints.WALL_FOLLOW_MAX_STEER), -mazeConstraints.WALL_FOLLOW_MAX_STEER)

        baseLeft = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT]
        baseRight = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT]

        steer = gyroSteer + wallSteer
        leftSpeed = int(max(min(100, baseLeft + steer), -100))
        rightSpeed = int(max(min(100, baseRight - steer), -100))
        stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed})


        if detectTileColor() == mazeEnums.tileType.BLACK and cameraBlackTileDetected:
            stmInstance.sts3032.stop()
            mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
            debugPrint("Black tile detected! Stopping movement. Starting escape maneuver.")
            startEscapeTime = time.time()
            stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -50, deviceEnums.Side.RIGHT: -50})
            while time.time() - startEscapeTime < practicalMoveTime:
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
            stmInstance.sts3032.stop()
            debugPrint(f"Escape maneuver complete.")
            return True, False
        else:
            tempTileColor = detectTileColor()
            if tempTileColor != mazeEnums.tileType.EMPTY:
                getTileColorDict[tempTileColor] += 1
            if tempTileColor == mazeEnums.tileType.RED:
                isRedTile = True

        escapeFlag = False
        timeBeforeEscape = time.time()
        if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT] or stmInstance.loadcell.getPressed()[deviceEnums.Side.RIGHT]:
            pressedSide = deviceEnums.Side.LEFT if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT] else deviceEnums.Side.RIGHT
            if stmInstance.tof.getDistance()[0] > 20:                    
                escapeFromObstacle(pressedSide, stmInstance)
                stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
                print((time.time() - oldTime)*np.cos(np.radians(abs(stmInstance.gyro.getValue().roll)))*0.1)
                practicalMoveTime -= 0.1
                escapeFlag = True
                time.sleep(0.1)

        if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC and isRamp:
            stmInstance.sts3032.stop()
            break
        if stmInstance.tof.getDistance()[0] < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
            stmInstance.sts3032.stop()
            break
        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            if isWallAhead[side]:
                victimInfo = stmInstance.unitv.getStatus()
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and (victimInfo[side] not in avoidVictim[side]) and consequentSearchRes[side] is None:
                    if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                        debugPrint("Skipping red victim on red tile")
                        continue
                    stmInstance.sts3032.stop()
                    t = time.time()
                    consequentSearchRes[side] = victimInfo[side]
                    mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360), vitimToWallType(consequentSearchRes[side]))
                    mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], consequentSearchRes[side])
                    print(f"Detected victim info ahead: {victimInfo}")
                    dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                    practicalMoveTime -= (time.time() - t)
            else:
                lastUpdateTime = stmInstance.unitv.getLastUpdateTime()[side]
                if (mazeConstraints.MOVE_STRAIGHT_SEC - practicalMoveTime < mazeConstraints.MOVE_STRAIGHT_SEC * 0.20):
                    victimInfo = stmInstance.unitv.getStatus()
                    if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                        getVictimDict[side][victimInfo[side]] += 1
                        print(f"Detected victim info during movement: {victimInfo}")
                if (practicalMoveTime - lastUpdateTime/1000) < mazeConstraints.MOVE_THRESHOLD_CM * 0.20:
                    victimInfo = stmInstance.unitv.getStatus()
                    if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] != mazeEnums.wallType.NO_WALL and (not mapInstance.isSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], victimInfo[side])):
                        if isRedTile and victimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                            debugPrint("Skipping red victim on red tile")
                            continue
                        stmInstance.sts3032.stop()
                        t = time.time()
                        print(f"Detected victim info during movement needing rescue kit drop: {victimInfo}")
                        dropRescueKit(stmInstance, mapInstance, victimInfo, side)
                        mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], victimInfo[side])
                        practicalMoveTime -= (time.time() - t)

        debugPrint(f"isramp: {isRamp}, roll: {stmInstance.gyro.getValue().roll} deg, practicalMoveTime: {practicalMoveTime} sec")
        practicalMoveTime += ((time.time() - oldTime) if not escapeFlag else (timeBeforeEscape - oldTime))*np.cos(np.radians(abs(roll))) * (1 if roll > 180 else 0.9)
        oldTime = time.time()
        print(f"moveTile loop time: {(time.time() - loop_start_time) * 1000:.1f} ms")
    stmInstance.sts3032.stop()
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
            currentDist = LiDAR.getCertainAngleDist(-heading + direction.value, scanPoints)
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
    stmInstance.sts3032.stop()   
    oldTime = time.time()   

    mapInstance.moveTo(direction)
    detectWall(lidar, mapInstance)
    maxVictimInfo = {side: max(getVictimDict[side], key=getVictimDict[side].get) if getVictimDict[side] else deviceEnums.UnitVStatus.NOTHING for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
        if mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] == mazeEnums.wallType.NO_WALL:
            continue
        if not (consequentSearchRes[side] is None):
            mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], consequentSearchRes[side])
        elif mapInstance.isSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], maxVictimInfo[side]) == False and maxVictimInfo[side] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] == mazeEnums.wallType.WALL:
            if isRedTile and maxVictimInfo[side] == deviceEnums.UnitVStatus.R_VICTIM:
                debugPrint("Skipping red victim on red tile")
                continue
            
            mapInstance.addSeenVictimType([mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)], maxVictimInfo[side])
            if maxVictimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                print(f"Decided victim on {side} side: {maxVictimInfo[side]}")
                dropRescueKit(stmInstance, mapInstance, maxVictimInfo, side)
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
            if getTileColorDict[t] > getTileColorDict[tileType] and getTileColorDict[t] >= max(mazeConstraints.MIN_TILE_DETECTION_THERESHOLD, nowMaxCount) and t != mazeEnums.tileType.EMPTY:
                tileType = t
                nowMaxCount = getTileColorDict[t]
        mapInstance.setTileType(tileType if tileType != mazeEnums.tileType.BLACK else mazeEnums.tileType.EMPTY)
    print(f"Tile color detection counts: {dict(getTileColorDict)}, decided tile type: {tileType}")

    if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
        stmInstance.buzzer.playMusic(buzzerSongs.swamp)
        time.sleep(5.2)
    if mapInstance.getTileType() != mazeEnums.tileType.EMPTY:
        print(f"Moved to {mapInstance.currentPosition}, Tile type: {mapInstance.getTileType()}, Wall types: {mapInstance.getWallType()}")
    stmInstance.sts3032.stop()
    return False, False

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stmInstance: stm.STM, lidar: ydlidar.CYdLidar) -> bool:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
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

