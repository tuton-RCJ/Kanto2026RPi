import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap
from .device import LiDAR, deviceConstraints, deviceEnums, stm, camera
import time
import numpy as np
import math
from collections import defaultdict
colorSensor = None
pr = None

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
        turnOnLED(stmInstance, color)
        time.sleep(intervalSec)
        turnOffLED(stmInstance)
        time.sleep(intervalSec)

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
        angle = (direction.value - currentDirVal + 360) % 360
        dist = LiDAR.getCertainAngleDist(angle, points)
        print(f"Direction: {direction}, Angle: {angle}, Distance: {dist} cm")
        if not mapInstance.getWallType()[direction].value >= mazeEnums.wallType.WALL.value:
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
    for i in range(needRescueKitCount):
        if mapInstance.nowRescueKitCount[side if not oppositeFlag else side.opposite()] >= 1:
            turnToCertainDirection((mapInstance.frontDirection.value + i*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.dropRescueKit(side if not oppositeFlag else side.opposite(), 1)
            stmInstance.rescuekitservo.dropRescueKit(1, side if not oppositeFlag else side.opposite())
            time.sleep(1)
        elif mapInstance.nowRescueKitCount[side.opposite() if not oppositeFlag else side] >= 1:
            turnToCertainDirection((mapInstance.frontDirection.value + 180 + i*mazeConstraints.TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS) % 360, stmInstance)
            mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)
            mapInstance.dropRescueKit(side.opposite() if not oppositeFlag else side, 1)
            stmInstance.rescuekitservo.dropRescueKit(1, side.opposite() if not oppositeFlag else side)
            oppositeFlag = not oppositeFlag
            time.sleep(1)
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
        return mazeEnums.wallType.UNKNOWN
    
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
            victimInfo = getVictimInfo(stmInstance)
            if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] != vitimToWallType(victimInfo[deviceEnums.Side.LEFT]) and mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL and leftFlag:
                print(f"Find victim on LEFT side: {victimInfo[deviceEnums.Side.LEFT]}")
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), vitimToWallType(victimInfo[deviceEnums.Side.LEFT]))
                dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.LEFT)
                print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                leftFlag = False

            if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] != vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]) and mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL and rightFlag:
                print(f"Find victim on RIGHT side: {victimInfo[deviceEnums.Side.RIGHT]}")
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]))
                dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
                print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                rightFlag = False
            stmInstance.update()
        mapInstance.addSeenCount()

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stmInstance: stm.STM, lidar: ydlidar.CYdLidar, firstTime: float) -> bool:
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
    debugPrint(f"Moving to {direction} from {mapInstance.currentPosition} facing {mapInstance.frontDirection}")
    if isUturn(mapInstance.frontDirection.value, direction.value, mapInstance):
        if (mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value) % 360)] == mazeEnums.wallType.WALL) or (mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)] == mazeEnums.wallType.WALL):
            turnToCertainDirection((mapInstance.frontDirection.value + 90) % 360, stmInstance)
            mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)
            rescueVictim(mapInstance, stmInstance)

        turnToCertainDirection(direction.value, stmInstance)
        mapInstance.frontDirection = direction
    else:
        turnToCertainDirection(direction.value, stmInstance)
        mapInstance.frontDirection = direction
    stmInstance.sts3032.stop()
    
    if (mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL) or (mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL):
        rescueVictim(mapInstance, stmInstance)        
        turnToCertainDirection(direction.value, stmInstance)
        
    stmInstance.update()
    if stmInstance.switch.getToggleSwitch1():
        stmInstance.sts3032.stop()
        return False, True
    points = LiDAR.getLiDARScan(lidar)
    nearestLiDARAngle = 0 if LiDAR.getCertainAngleDist(0, points) < LiDAR.getCertainAngleDist(180, points) else 180
    oldDist = LiDAR.getCertainAngleDist(nearestLiDARAngle-stmInstance.gyro.getValue().heading+direction.value, points)
    stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
    littleFowardFlag = False
    isRamp = False
    startTime = time.time()
    practicalMoveTime = 0.0
    oldTime = startTime
    getVictimDict = {deviceEnums.Side.LEFT: defaultdict(int), deviceEnums.Side.RIGHT: defaultdict(int)}
    getTileColorDict = defaultdict(int)
    cameraBlackTileDetected = False
    while True:

        isBlackTileByCam = camera.detectTileColor() == "BLACK"
        cameraBlackTileDetected = cameraBlackTileDetected or isBlackTileByCam

        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return False, True
        scanPoints = LiDAR.getLiDARScan(lidar)
        currentDist = LiDAR.getCertainAngleDist(nearestLiDARAngle-stmInstance.gyro.getValue().heading+direction.value, scanPoints)
        turnAngle = regulationAngle(stmInstance.gyro.getValue().heading - direction.value)
        leftWallDist = LiDAR.getCertainAngleDist(90-stmInstance.gyro.getValue().heading+direction.value, scanPoints)
        rightWallDist = LiDAR.getCertainAngleDist(270-stmInstance.gyro.getValue().heading+direction.value, scanPoints)
        targetDistDiff = math.sqrt(max(min(1-abs(oldDist - currentDist)/30, (stmInstance.tof.getDistance()[0]-15)/15),0))    
        if not isRamp and (90 > min(stmInstance.gyro.getValue().roll, 360 - stmInstance.gyro.getValue().roll) > mazeConstraints.RAMP_DEG_THRESHOLD):
            isRamp = True
            print(f"Ramp detected! Pitch: {stmInstance.gyro.getValue().roll} deg")
        if not isRamp:
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

            steer = gyroSteer + wallSteer

            baseLeft = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT] * targetDistDiff + 20
            baseRight = mazeConstraints.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT] * targetDistDiff + 20

            leftSpeed = int(max(min(100, baseLeft + steer), -100))
            rightSpeed = int(max(min(100, baseRight - steer), -100))
            stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: leftSpeed, deviceEnums.Side.RIGHT: rightSpeed})
        else:
            stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_MAX_SPEED)
        if detectTileColor() == mazeEnums.tileType.BLACK and cameraBlackTileDetected:
            stmInstance.sts3032.stop()
            mapInstance.setTileType(mazeEnums.tileType.BLACK, direction=direction)
            debugPrint("Black tile detected! Stopping movement. Starting escape maneuver.")
            while (abs(LiDAR.getCertainAngleDist(nearestLiDARAngle, LiDAR.getLiDARScan(lidar)) - oldDist) > mazeConstraints.TOF_BLACK_TILE_ESCAPE_DISTANCE_CM):
                stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30})
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    stmInstance.sts3032.stop()
                    return False, True
            debugPrint(f"Escape maneuver complete. Current Distance: {LiDAR.getCertainAngleDist(nearestLiDARAngle, LiDAR.getLiDARScan(lidar))} cm")
            return True, False
        else:
            tempTileColor = detectTileColor()
            if tempTileColor != mazeEnums.tileType.EMPTY:
                getTileColorDict[tempTileColor] += 1
        
        if  (abs((oldDist) - (currentDist))> mazeConstraints.MOVE_THRESHOLD_CM or LiDAR.getCertainAngleDist(-stmInstance.gyro.getValue().heading + direction.value, scanPoints) < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM) and (not 90 > min(stmInstance.gyro.getValue().pitch, 360 - stmInstance.gyro.getValue().pitch) > mazeConstraints.RAMP_DEG_THRESHOLD) and (not isRamp):
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
            practicalMoveTime -= (time.time() - oldTime)*np.cos(np.radians(abs(stmInstance.gyro.getValue().roll)))*0.2
            escapeFlag = True

        if practicalMoveTime > mazeConstraints.MOVE_STRAIGHT_SEC and isRamp:
            stmInstance.sts3032.stop()
            break
        if (30 - abs(oldDist - currentDist) < mazeConstraints.MOVE_THRESHOLD_CM * 0.2) and not isRamp:
            victimInfo = stmInstance.unitv.getStatus()
            for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
                if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    getVictimDict[side][victimInfo[side]] += 1
                    debugPrint(f"Detected victim info during movement: {victimInfo}")

        debugPrint(f"isramp: {isRamp}, pitch: {stmInstance.gyro.getValue().roll} deg, practicalMoveTime: {practicalMoveTime} sec, currentDist: {currentDist} cm, oldDist: {oldDist} cm")
        practicalMoveTime += ((time.time() - oldTime) if not escapeFlag else (timeBeforeEscape - oldTime))*np.cos(np.radians(abs(stmInstance.gyro.getValue().roll)))
        oldTime = time.time()


    if littleFowardFlag:
        stmInstance.sts3032.setMotorSpeed(mazeConstraints.GO_STRAIGHT_LOW_SPEED)
        debugPrint("Little forward to adjust position")
        while True:
            stmInstance.update()
            if stmInstance.switch.getToggleSwitch1():
                stmInstance.sts3032.stop()
                return False, True
            currentDist = stmInstance.tof.getDistance()[0]
            if currentDist < mazeConstraints.MOVE_STRAIGHT_THRESHOLD_CM:
                break
    stmInstance.sts3032.stop()   
    oldTime = time.time()   
    
    while time.time() - oldTime < 0.2:
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():
            stmInstance.sts3032.stop()
            return False, True
        victimInfo = getVictimInfo(stmInstance)
        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            if victimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                getVictimDict[side][victimInfo[side]] += 1

    mapInstance.moveTo(direction)    
    detectWall(lidar, mapInstance)
    maxVictimInfo = {side: max(getVictimDict[side], key=getVictimDict[side].get) if getVictimDict[side] else deviceEnums.UnitVStatus.NOTHING for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]}
    if (maxVictimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL and mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] <= 1) or (maxVictimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL and mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] <= 1):
        for side in [deviceEnums.Side.LEFT, deviceEnums.Side.RIGHT]:
            if mapInstance.getSeenCount()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] <= 1 and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360)] == mazeEnums.wallType.WALL:
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + (90 if side == deviceEnums.Side.LEFT else 270)) % 360), vitimToWallType(maxVictimInfo[side]))
                if maxVictimInfo[side] != deviceEnums.UnitVStatus.NOTHING:
                    print(f"Decided victim on {side} side: {maxVictimInfo[side]}")
                    dropRescueKit(stmInstance, mapInstance, maxVictimInfo, side)
        mapInstance.addSeenCount()
    tileType = mazeEnums.tileType.EMPTY
    nowMaxCount = 0
    if isSilverTile():
        mapInstance.setTileType(mazeEnums.tileType.SILVER)
        tileType = mazeEnums.tileType.SILVER
    else:
        for t in list(getTileColorDict.keys()):
            if t.value == "E":
                continue
            if getTileColorDict[t] > getTileColorDict[tileType] and getTileColorDict[t] >= max(mazeConstraints.MIN_TILE_DETECTION_THERESHOLD, nowMaxCount) and t != mazeEnums.tileType.EMPTY:
                tileType = t
                nowMaxCount = getTileColorDict[t]
        mapInstance.setTileType(tileType)
    print(f"Tile color detection counts: {dict(getTileColorDict)}, decided tile type: {tileType}")

    if mapInstance.getTileType() == mazeEnums.tileType.BLUE:
        time.sleep(5)
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

