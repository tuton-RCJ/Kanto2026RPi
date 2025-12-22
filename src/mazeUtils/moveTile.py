import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap
from .device import LiDAR, deviceConstraints, deviceEnums, stm
import time
import numpy as np

colorSensor = None

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
        elif ((mazeConstraints.SILVERTILE_RGB[1][0] < r < mazeConstraints.SILVERTILE_RGB[0][0]) and
              (mazeConstraints.SILVERTILE_RGB[1][1] < g < mazeConstraints.SILVERTILE_RGB[0][1]) and
              (mazeConstraints.SILVERTILE_RGB[1][2] < b < mazeConstraints.SILVERTILE_RGB[0][2])):
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

def turnToCertainDirection(targetDir: int, stmInstance: stm.STM) -> None:
    """
    @brief 指定した絶対方向に向く
    @param targetDir: 目標の絶対方向 (0-359)
    @param stmInstance: 通信に使用する STM インスタンス
    """

    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return
    if not mazeConstraints.USE_PD_FOR_TURNING: 
        turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)

        stmInstance.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(50)
        debugPrint("current heading:", stmInstance.gyro.getValue().heading, "target:", targetDir)
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
        assert abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) <= mazeConstraints.TURN_THRESHOLD_DEG, f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
        stmInstance.sts3032.stop()

        debugPrint("stopped turning at heading:", stmInstance.gyro.getValue().heading, "diff:" , abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)))
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return
        
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX:
            stmInstance.sts3032.turnRight(5) if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir) == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(5)
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
        stmInstance.sts3032.stop()
    else: # PD制御
        turnAngle = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        debugPrint(f"Turning to {targetDir} deg, current heading: {stmInstance.gyro.getValue().heading} deg, turnAngle: {turnAngle} deg")

        oldError = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
        while abs(regulationAngle(stmInstance.gyro.getValue().heading - targetDir)) > mazeConstraints.TURN_THRESHOLD_DEG_FIX:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return
            error = regulationAngle(stmInstance.gyro.getValue().heading - targetDir)
            deribative = error - oldError
            oldError = error
            turnSpeed = mazeConstraints.TURN_P * error + mazeConstraints.TURN_D * deribative
            turnSpeed = max(min(turnSpeed, 100), -100)
            turnSpeed = turnSpeed if abs(turnSpeed) >= 5 else (5 if turnSpeed > 0 else -5)
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
    
def detectWall(lidar: ydlidar.CYdLidar, mapInstance: mazeMap.mazeMap) -> None:
    points = LiDAR.getLiDARScan(lidar)
    currentDirVal = mapInstance.frontDirection.value
    for direction in mazeEnums.absDirection:
        angle = (direction.value - currentDirVal) % 360
        dist = LiDAR.getCertainAngleDist(angle, points)
        debugPrint(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")

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
    victimData = stmInstance.unitv.getStatus()
    return victimData

def dropRescueKit(stmInstance: stm.STM, mapInstance: mazeMap.mazeMap, victimInfo: dict[deviceEnums.Side,deviceEnums.UnitVStatus], side: deviceEnums.Side) -> None:
    """
    @brief 指定した側に救助キットを投下する
    @param stmInstance: 通信に使用する STM インスタンス
    @param side: 救助キットを投下する側
    """
    needRescueKitCount = (victimInfo[side].value - 1)%3 
    if mapInstance.nowRescueKitCount[side] >= needRescueKitCount and needRescueKitCount > 0:
        mapInstance.dropRescueKit(side, needRescueKitCount)
        stmInstance.rescuekitservo.dropRescueKit(needRescueKitCount, side)
    elif mapInstance.nowRescueKitCount[side.opposite()] >= needRescueKitCount and needRescueKitCount > 0:
        turnToCertainDirection((mapInstance.frontDirection.value + 180) % 360, stmInstance)
        mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
        mapInstance.dropRescueKit(side.opposite(), needRescueKitCount)
        stmInstance.rescuekitservo.dropRescueKit(needRescueKitCount, side.opposite())
    else:
        debugPrint(f"Not enough rescue kits to drop on {side} side.")

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stm: stm.STM, lidar: ydlidar.CYdLidar, firstTime: float) -> bool:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """

    stm.update()
    if stm.switch.getToggleSwitch1():
        stm.sts3032.stop()
        return False, False
    debugPrint(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")

    turnToCertainDirection(direction.value, stm)

    mapInstance.frontDirection = direction
    
    stm.update()
    if stm.switch.getToggleSwitch1():
        stm.sts3032.stop()
        return False, False
    nearestToFIndex = 0 if stm.tof.getDistance()[0] < stm.tof.getDistance()[2] else 2
    oldDist = stm.tof.getDistance()[nearestToFIndex]
    stm.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    isRamp = False
    startTime = time.time()
    practicalMoveTime = 0.0
    oldTime = startTime
    while True:

        stm.update()
        if stm.switch.getToggleSwitch1():
            stm.sts3032.stop()
            return False, False
        currentDist = stm.tof.getDistance()[nearestToFIndex]
        
        if detectTileColor() == mazeEnums.tileType.BLACK:
            stm.sts3032.stop()
            mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
            debugPrint("Black tile detected! Stopping movement. Starting escape maneuver.")
            while (abs(stm.tof.getDistance()[nearestToFIndex] - oldDist) > mazeConstraints.TOF_BLACK_TILE_ESCAPE_DISTANCE_CM) and firstTime + mazeConstraints.MOVETILE_TIMEOUT_SEC > time.time():
                stm.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30})
                stm.update()
                if stm.switch.getToggleSwitch1():
                    stm.sts3032.stop()
                    return
            debugPrint(f"Escape maneuver complete. Current Distance: {stm.tof.getDistance()[nearestToFIndex]} cm")
            return True, False
        
        if  abs((oldDist) - (currentDist))> mazeConstraints.MOVE_THRESHOLD_CM or stm.tof.getDistance()[0] < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
            stm.sts3032.stop()
            if mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM < currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM + 10:
                littleFowardFlag = True
            break

        if stm.loadcell.getPressed()[deviceEnums.Side.LEFT] or stm.loadcell.getPressed()[deviceEnums.Side.RIGHT]:
            pressedSide = deviceEnums.Side.LEFT if stm.loadcell.getPressed()[deviceEnums.Side.LEFT] else deviceEnums.Side.RIGHT
            escapeFromObstacle(pressedSide, stm)
            stm.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
        if not isRamp and 90 > stm.gyro.getValue().pitch > mazeConstraints.RAMP_DEG_THRESHOLD:
            isRamp = True
            debugPrint(f"Ramp detected! Pitch: {stm.gyro.getValue().pitch} deg")
        if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC and isRamp:
            stm.sts3032.stop()
            break

        debugPrint(f"isramp: {isRamp}, pitch: {stm.gyro.getValue().pitch} deg, practicalMoveTime: {practicalMoveTime} sec, currentDist: {currentDist} cm, oldDist: {oldDist} cm")
        practicalMoveTime += (time.time() - oldTime)*np.cos(np.radians(abs(stm.gyro.getValue().pitch)))
        oldTime = time.time()


    if littleFowardFlag:
        stm.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")
        while firstTime + mazeConstraints.MOVETILE_TIMEOUT_SEC > time.time():
            stm.update()
            if stm.switch.getToggleSwitch1():
                stm.sts3032.stop()
                return False, False
            currentDist = stm.tof.getDistance()[0]
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
            
    if firstTime + mazeConstraints.MOVETILE_TIMEOUT_SEC <= time.time():
        debugPrint("Move timeout reached.")
        return False, True
    stm.sts3032.stop()
    return False, False

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
        return mazeEnums.wallType.UNKNOWN

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stm: stm.STM, lidar: ydlidar.CYdLidar) -> bool:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """
    firstTime = time.time()
    isBlack, timeoutReached = moveTile(direction, mapInstance, stm, lidar, firstTime)

    if not isBlack:
        mapInstance.moveTo(direction)    
        detectWall(lidar, mapInstance)
        tileType = detectTileColor()
        mapInstance.setTileType(tileType)
        victimInfo = getVictimInfo(stm)
        if tileType == mazeEnums.tileType.BLUE:
            time.sleep(5)
        if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] != vitimToWallType(victimInfo[deviceEnums.Side.LEFT]):
            mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), vitimToWallType(victimInfo[deviceEnums.Side.LEFT]))
            dropRescueKit(stm, mapInstance, victimInfo, deviceEnums.Side.LEFT)
        if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] != vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]):
            mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]))
            dropRescueKit(stm, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
        debugPrint("Moved to", mapInstance.currentPosition, "facing", mapInstance.frontDirection, "Tile type:", tileType)
    stm.update()
    if stm.switch.getToggleSwitch1():
        stm.sts3032.stop()
        return
    return isBlack, timeoutReached

def turnOnLED(stmInstance: stm.STM) -> None:
    """
    @brief LEDを点灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(255,255,255)

def turnOffLED(stmInstance: stm.STM) -> None:
    """
    @brief LEDを消灯する
    @param stmInstance: 通信に使用する STM インスタンス
    """
    stmInstance.led.setColor(0,0,0)

def flashLED(stmInstance: stm.STM, loopCount: int, intervalSec: float) -> None:
    """
    @brief LEDを点滅させる
    @param stmInstance: 通信に使用する STM インスタンス
    @param durationSec: 点滅させる時間 (秒)
    @param intervalSec: 点灯と消灯の間隔 (秒)
    """
    for _ in range(loopCount):
        turnOnLED(stmInstance)
        time.sleep(intervalSec)
        turnOffLED(stmInstance)
        time.sleep(intervalSec)