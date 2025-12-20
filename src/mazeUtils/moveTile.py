import ydlidar
from . import mazeConsrains, mazeEnums, mazeMap
from .device import LiDAR, deviceEnums, stm, deviceConstrains
import time

colorSensor = None

def debugPrint(*message: object) -> None:
    if mazeConsrains.DEBUG_MODE:
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
        debugPrint(f"ColorSensor RGB: R={r}, G={g}, B={b}")
        if ((mazeConsrains.BLACKTILE_RGB[1][0] < r < mazeConsrains.BLACKTILE_RGB[0][0]) and
            (mazeConsrains.BLACKTILE_RGB[1][1] < g < mazeConsrains.BLACKTILE_RGB[0][1])and
            (mazeConsrains.BLACKTILE_RGB[1][2] < b < mazeConsrains.BLACKTILE_RGB[0][2])):
            return mazeEnums.tileType.BLACK
        elif ((mazeConsrains.BLUETILE_RGB[1][0] < r < mazeConsrains.BLUETILE_RGB[0][0]) and
              (mazeConsrains.BLUETILE_RGB[1][1] < g < mazeConsrains.BLUETILE_RGB[0][1]) and
              (mazeConsrains.BLUETILE_RGB[1][2] < b < mazeConsrains.BLUETILE_RGB[0][2])):
            return mazeEnums.tileType.BLUE
        elif ((mazeConsrains.SILVERTILE_RGB[1][0] < r < mazeConsrains.SILVERTILE_RGB[0][0]) and
              (mazeConsrains.SILVERTILE_RGB[1][1] < g < mazeConsrains.SILVERTILE_RGB[0][1]) and
              (mazeConsrains.SILVERTILE_RGB[1][2] < b < mazeConsrains.SILVERTILE_RGB[0][2])):
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

def turnToCertainDirection(targetDir: int, stmInstance: stm.STM) -> None:
    """
    @brief 指定した絶対方向に向く
    @param targetDir: 目標の絶対方向 (0-359)
    @param stmInstance: 通信に使用する STM インスタンス
    """

    stmInstance.update()
    if not mazeConsrains.USE_PD_FOR_TURNING:
        turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, targetDir)
        stmInstance.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(50)
        debugPrint("current heading:", stmInstance.gyro.getValue().heading, "target:", targetDir)
        while abs(stmInstance.gyro.getValue().heading - targetDir) > mazeConsrains.TURN_THRESHOLD_DEG:
            stmInstance.update()
        assert abs(stmInstance.gyro.getValue().heading - targetDir) <= mazeConsrains.TURN_THRESHOLD_DEG, f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {targetDir}"
        stmInstance.sts3032.stop()

        debugPrint("stopped turning at heading:", stmInstance.gyro.getValue().heading, "diff:" , abs(stmInstance.gyro.getValue().heading - targetDir))
        firstFlag = True
        stmInstance.update()
        
        while abs(stmInstance.gyro.getValue().heading - targetDir) > mazeConsrains.TURN_THRESHOLD_DEG_FIX:
            if firstFlag:
                debugPrint("Fine adjustment")
                stmInstance.sts3032.turnRight(5) if getTurnDirection(stmInstance.gyro.getValue().heading, targetDir) == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(5)
                firstFlag = False
            stmInstance.update()
        stmInstance.sts3032.stop()
    else:
        turnAngle = stmInstance.gyro.getValue().heading - targetDir
        if turnAngle > 180:
            turnAngle -= 360
        if turnAngle < -180:
            turnAngle += 360
        debugPrint(f"Turning to {targetDir} deg, current heading: {stmInstance.gyro.getValue().heading} deg, turnAngle: {turnAngle} deg")
        # pd
        oldError = stmInstance.gyro.getValue().heading - targetDir
        while abs(stmInstance.gyro.getValue().heading - targetDir) > mazeConsrains.TURN_THRESHOLD_DEG:
            stmInstance.update()
            error = (stmInstance.gyro.getValue().heading - targetDir + 180) % 360 - 180
            deribative = error - oldError
            oldError = error
            turnSpeed = mazeConsrains.TURN_P * error + mazeConsrains.TURN_D * deribative
            turnSpeed = max(min(turnSpeed, 50), -50)
            stmInstance.sts3032.turnRight(abs(int(turnSpeed))) if turnSpeed > 0 else stmInstance.sts3032.turnLeft(abs(int(turnSpeed)))
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

        if dist < mazeConsrains.WALL_DETECTION_THRESHOLD_CM:
            mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
        else:
            mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

def detectTileType(mapInstance: mazeMap.mazeMap) -> None: #TODO: implement this
    mapInstance.setTileType(mazeEnums.tileType.EMPTY)
    pass

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
        turnToCertainDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360, stmInstance)
        mapInstance.dropRescueKit(side.opposite(), needRescueKitCount)
        stmInstance.rescuekitservo.dropRescueKit(needRescueKitCount, side.opposite())
    else:
        debugPrint(f"Not enough rescue kits to drop on {side} side.")

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stm: stm.STM, lidar: ydlidar.CYdLidar) -> None:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """

    stm.update()
    debugPrint(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")

    if mazeConsrains.USE_TURN_METHOD == mazeEnums.turnMethod.ONLY_GYRO:
            turnDirection = getTurnDirection(mapInstance.frontDirection.value, direction.value)
            debugPrint(f"Turning from {mapInstance.frontDirection} to {direction}, turnDirection: {turnDirection}")
            stm.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stm.sts3032.turnLeft(50)
            debugPrint("current heading:", stm.gyro.getValue().heading, "target:", direction.value)
            while abs(stm.gyro.getValue().heading - direction.value) > mazeConsrains.TURN_THRESHOLD_DEG:
                stm.update()
            assert abs(stm.gyro.getValue().heading - direction.value) <= mazeConsrains.TURN_THRESHOLD_DEG, f"Gyro turn failed to reach target heading, current: {stm.gyro.getValue().heading}, target: {direction.value}"
            stm.sts3032.stop()

            debugPrint("stopped turning at heading:", stm.gyro.getValue().heading, "diff:" , abs(stm.gyro.getValue().heading - direction.value))
            firstFlag = True
            stm.update()
            
            while abs(stm.gyro.getValue().heading - direction.value) > mazeConsrains.TURN_THRESHOLD_DEG_FIX:
                if firstFlag:
                    debugPrint("Fine adjustment")
                    stm.sts3032.turnRight(5) if getTurnDirection(stm.gyro.getValue().heading, direction.value) == mazeEnums.turnDirection.RIGHT else stm.sts3032.turnLeft(5)
                    firstFlag = False
                stm.update()

            stm.update()
            debugPrint(f"Turned to heading: {stm.gyro.getValue().heading} deg")

    mapInstance.frontDirection = direction
    
    stm.update()
    nearestToFIndex = 0 if stm.tof.getDistance()[0] < stm.tof.getDistance()[2] else 2
    oldDist = stm.tof.getDistance()[nearestToFIndex]
    stm.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    while True:

        stm.update()
        currentDist = stm.tof.getDistance()[nearestToFIndex]
        
        if detectTileColor() == mazeEnums.tileType.BLACK:
            stm.sts3032.stop()
            mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
            debugPrint("Black tile detected! Stopping movement. Starting escape maneuver.")
            while abs(stm.tof.getDistance()[nearestToFIndex] - oldDist) > mazeConsrains.TOF_BLACK_TILE_ESCAPE_DISTANCE_CM:
                stm.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30})
                stm.update()
            debugPrint(f"Escape maneuver complete. Current Distance: {stm.tof.getDistance()[nearestToFIndex]} cm")
            return True
        
        if  abs((oldDist) - (currentDist))> mazeConsrains.MOVE_THRESHOLD_CM or stm.tof.getDistance()[0] < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM:
            stm.sts3032.stop()
            if mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM < currentDist < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM + 10:
                littleFowardFlag = True
            break

        if stm.loadcell.getPressed()[deviceEnums.Side.LEFT] or stm.loadcell.getPressed()[deviceEnums.Side.RIGHT]:
            pressedSide = deviceEnums.Side.LEFT if stm.loadcell.getPressed()[deviceEnums.Side.LEFT] else deviceEnums.Side.RIGHT
            escapeFromObstacle(pressedSide, stm)
            stm.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_MAX_SPEED)

    debugPrint(f"Moved forward. Old Distance: {oldDist} cm, Current Distance: {currentDist} cm")

    if littleFowardFlag:
        stm.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")
        while True:
            stm.update()
            currentDist = stm.tof.getDistance()[0]
            if currentDist < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM:
                break
        debugPrint(f"Final adjustment done. Current Distance: {currentDist} cm")


    stm.sts3032.stop()
    return False

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

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stm: stm.STM, lidar: ydlidar.CYdLidar) -> None:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """
    isBlack = moveTile(direction, mapInstance, stm, lidar)

    if not isBlack:
        mapInstance.moveTo(direction)    
        detectWall(lidar, mapInstance)
        tileType = detectTileColor()
        mapInstance.setTileType(tileType)
        victimInfo = getVictimInfo(stm)
        if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)) != vitimToWallType(victimInfo[deviceEnums.Side.LEFT]):
            dropRescueKit(stm, mapInstance, victimInfo, deviceEnums.Side.LEFT)
            mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), vitimToWallType(victimInfo[deviceEnums.Side.LEFT]))
        if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)) != vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]):
            dropRescueKit(stm, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
            mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]))
    return isBlack