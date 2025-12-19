import time

import ydlidar

from . import mazeConsrains, mazeEnums, mazeMap
from .device import LiDAR, deviceEnums, stm


colorSensor = None


def detectBlackTile() -> bool:
    """
    @brief カラーセンサで黒タイルを検出する
    @return: 黒タイルが検出されたら True、そうでなければ False
    """
    global colorSensor
    if colorSensor is None:
        # Lazy import to avoid import/serial errors on non-RPi environments.
        from .device import colorsensor

        colorSensor = colorsensor.ColorSensor()

    try:
        if not colorSensor.update():
            return False
        r, g, b = colorSensor._colorRGB
    except Exception as e:
        print(f"ColorSensor read failed: {e}")
        return False

    brightness = (r + g + b) / 3.0
    return brightness <= mazeEnums.BLACK_TILE_BRIGHTNESS_THRESHOLD

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

    
def getTurnDirection(fromDir: float, toDir: float) -> mazeEnums.turnDirection:
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
        print(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")

        if dist < mazeConsrains.WALL_DETECTION_THRESHOLD_CM:
            mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
        else:
            mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

def detectTileType(mapInstance: mazeMap.mazeMap) -> None: #TODO: implement this
    mapInstance.setTileType(mazeEnums.tileType.EMPTY)


def turnToDirection(stmInstance: stm.STM, direction: mazeEnums.absDirection) -> None:
    turnDirection = getTurnDirection(stmInstance.gyro.getValue().heading, direction.value)
    stmInstance.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(50)

    while abs(stmInstance.gyro.getValue().heading - direction.value) > mazeConsrains.TURN_THRESHOLD_DEG:
        stmInstance.update()
        print(stmInstance.gyro.getValue().heading)

    assert abs(stmInstance.gyro.getValue().heading - direction.value) <= mazeConsrains.TURN_THRESHOLD_DEG, (
        f"Gyro turn failed to reach target heading, current: {stmInstance.gyro.getValue().heading}, target: {direction.value}"
    )
    stmInstance.sts3032.stop()

    print(
        "stopped turning at heading:",
        stmInstance.gyro.getValue().heading,
        "diff:",
        abs(stmInstance.gyro.getValue().heading - direction.value),
    )

    firstFlag = True
    stmInstance.update()
    while abs(stmInstance.gyro.getValue().heading - direction.value) > mazeConsrains.TURN_THRESHOLD_DEG_FIX:
        if firstFlag:
            print("Fine adjustment")
            stmInstance.sts3032.turnRight(10) if getTurnDirection(stmInstance.gyro.getValue().heading, direction.value) == mazeEnums.turnDirection.RIGHT else stmInstance.sts3032.turnLeft(10)
            firstFlag = False
        stmInstance.update()


def driveForwardUntilTile(stmInstance: stm.STM) -> bool:
    """
    @brief 次のタイルまで前進する
    @param stmInstance: 通信に使用する STM インスタンス
    @return: 黒タイルが検出されたら True、そうでなければ False
    """
    stmInstance.update()
    nearestToFIndex = 0 if stmInstance.tof.getDistance()[0] < stmInstance.tof.getDistance()[2] else 2
    oldDist = stmInstance.tof.getDistance()[nearestToFIndex]
    stmInstance.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_MAX_SPEED)

    littleForwardFlag = False
    while True:
        stmInstance.update()
        currentDist = stmInstance.tof.getDistance()[nearestToFIndex]

        if detectBlackTile():
            stmInstance.sts3032.stop()
            return True

        if (
            abs(oldDist - currentDist) > mazeConsrains.MOVE_THRESHOLD_CM
            or stmInstance.tof.getDistance()[0] < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM
        ):
            stmInstance.sts3032.stop()
            if mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM < currentDist < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM + 10:
                littleForwardFlag = True
            break

        if (
            stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT]
            or stmInstance.loadcell.getPressed()[deviceEnums.Side.RIGHT]
        ):
            pressedSide = (deviceEnums.Side.LEFT if stmInstance.loadcell.getPressed()[deviceEnums.Side.LEFT] else deviceEnums.Side.RIGHT)
            escapeFromObstacle(pressedSide, stmInstance)
            stmInstance.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_MAX_SPEED)

        print(f"Moved forward. Old Distance: {oldDist} cm, Current Distance: {currentDist} cm")

        if littleForwardFlag:
            stmInstance.sts3032.setMotorSpeed(mazeConsrains.GO_STRAIGHT_LOW_SPEED)
            print("Little forward to adjust position")
            while True:
                stmInstance.update()
                currentDist = stmInstance.tof.getDistance()[0]
                if currentDist < mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM:
                    break
            print(f"Final adjustment done. Current Distance: {currentDist} cm")

    stmInstance.sts3032.stop()
    return False

def backToPreviousTile(stmInstance: stm.STM, driveDist: float) -> None:
    """
    @brief 指定された距離後退するか、後ろの壁との距離が mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM 以下になるまで後退する
    @param stmInstance: 通信に使用する STM インスタンス
    @param driveDist: 後退する距離 (cm)
    """
    stmInstance.update()
    nearstToFIndex = 0 if stmInstance.tof.getDistance()[0] < stmInstance.tof.getDistance()[2] else 2
    oldDist = stmInstance.tof.getDistance()[nearstToFIndex]    
    stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -mazeConsrains.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT],
                                        deviceEnums.Side.RIGHT: -mazeConsrains.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT]})
    while stmInstance.tof.getDistance()[2] > mazeConsrains.MOVE_STRAIGHT_THRESHOLD_CM or abs(oldDist - stmInstance.tof.getDistance()[nearstToFIndex]) < driveDist:
        stmInstance.update()
        currentDist = stmInstance.tof.getDistance()[nearstToFIndex]

    print(f"Moved backward. Old Distance: {oldDist} cm, Current Distance: {currentDist} cm")
    stmInstance.sts3032.stop()

    

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stm: stm.STM, lidar: ydlidar.CYdLidar) -> bool:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """

    stm.update()
    print(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")

    turnDirection = getTurnDirection(mapInstance.frontDirection.value, direction.value)
    print(f"Turning from {mapInstance.frontDirection} to {direction}, turnDirection: {turnDirection}")
    print("current heading:", stm.gyro.getValue().heading, "target:", direction.value)

    turnToDirection(stm, direction)

    stm.update()
    print(f"Turned to heading: {stm.gyro.getValue().heading} deg")

    mapInstance.frontDirection = direction

    blackDetected = driveForwardUntilTile(stm)
    return blackDetected

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stm: stm.STM, lidar: ydlidar.CYdLidar) -> None:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """
    blackDetected = moveTile(direction, mapInstance, stm, lidar)
    if not blackDetected:
        mapInstance.moveTo(direction)
    else:
        mapInstance
        mapInstance.setTileType(mazeEnums.tileType.BLACK, mapInstance.getNextPosition(direction))
        return

    detectWall(lidar, mapInstance)
    detectTileType(mapInstance)
    return

