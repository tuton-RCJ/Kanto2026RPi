import ydlidar
from . import mazeConsrains, mazeEnums, mazeMap
from .device import LiDAR, deviceEnums, stm, deviceConstrains
import time
def escpapeObstacle():
    pass

def detectBlackTile():
    pass

def detectWall(lidar: ydlidar.CYdLidar, mapInstance: mazeMap.mazeMap) -> None:
    points = LiDAR.getLiDARScan(lidar)
    for direction in mazeEnums.absDirection:
        angle = direction.value
        dist = LiDAR.getCertainAngleDist(angle, points)
        print(f"Direction: {direction}, Distance: {dist} cm")

        if dist < mazeConsrains.WALL_DETECTION_THRESHOLD_CM:
            mapInstance.setWallType(direction, mazeEnums.wallType.WALL)
        else:
            mapInstance.setWallType(direction, mazeEnums.wallType.NO_WALL)

def detectTileType(mapInstance: mazeMap.mazeMap) -> None: #TODO: implement this
    mapInstance.setTileType(mazeEnums.tileType.EMPTY)
    pass

def moveTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap, stm: stm.STM, lidar: ydlidar.CYdLidar) -> None:
    """
    @brief direction の方向へ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """

    stm.update()

    if mapInstance.frontDirection != direction:
        if mazeConsrains.USE_TURN_METHOD == mazeEnums.turnMethod.ONLY_LiDAR:
                points = LiDAR.getLiDARScan(lidar)
                minDist = 1e9
                minDirection = None
                
                for d in mazeEnums.absDirection:
                    dist = LiDAR.getCertainAngleDist(d.value, points)
                    if dist < minDist:
                        minDist = dist
                        minDirection = d
                
                relativeAngle = LiDAR.getRelativeAngle(minDirection.value, 5, points)
                print(f"Initial Relative Angle: {relativeAngle} deg")
                print(f"Min Direction: {minDirection}, Min Distance: {minDist} cm")
                print(f"Turning from {mapInstance.frontDirection} to {direction}")
                print(f"Angle Difference: {(mapInstance.frontDirection - direction).value} deg")

                if (mapInstance.frontDirection - direction).value == 270:
                    turnDirection = mazeEnums.turnDirection.RIGHT
                else:
                    turnDirection = mazeEnums.turnDirection.LEFT

                stm.sts3032.turnRight(50) if turnDirection == mazeEnums.turnDirection.RIGHT else stm.sts3032.turnLeft(50)
                t = time.time()

                while (time.time() - t) < deviceConstrains.TURN_SEC*(((mapInstance.frontDirection - direction).value + abs(relativeAngle))/90):
                    pass
                stm.sts3032.stop()

                relativeAngle = LiDAR.getRelativeAngle(direction.value, 5, points)
        
        if mazeConsrains.USE_TURN_METHOD == mazeEnums.turnMethod.ONLY_GYRO:
                            
                turnDirection = mazeEnums.turnDirection.RIGHT if (stm.gyro.getAngleZ() - direction.value) > 0 else mazeEnums.turnDirection.LEFT
                stm.sts3032.turnRight() if turnDirection == mazeEnums.turnDirection.RIGHT else stm.sts3032.turnLeft()

                while abs(stm.gyro.getAngleZ()  - direction.value) > mazeConsrains.TURN_THRESHOLD_DEG:
                    stm.update()
                stm.sts3032.stop()
    
    points = LiDAR.getLiDARScan(lidar)
    
    if mazeConsrains.USE_MOVE_METHOD == mazeEnums.moveMethod.SEE_FRONT:
        oldDist = LiDAR.getCertainAngleDist(0, points)
        while True:
            points = LiDAR.getLiDARScan(lidar)
            currentDist = LiDAR.getCertainAngleDist(0, points)
            stm.update()
            
            if any([p for p in stm.loadcell.getPressed().values()]):
                escpapeObstacle()
                return 
            
            if detectBlackTile():
                stm.sts3032.stop()
                mazeMap.setTileType(mazeEnums.tileType.BLACK)
                return
            
            if (oldDist) - (currentDist) > mazeConsrains.MOVE_THRESHOLD_CM:
                stm.sts3032.stop()
                break
            print(oldDist, currentDist)
            print(f"Current Dist: {currentDist} cm")
            

            sideDist = LiDAR.getCertainAngleDist([-90, 90], points)
            diff = sideDist[1] - sideDist[0]
            leftSpeed = mazeConsrains.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.LEFT] - diff * mazeConsrains.P_GAIN 
            rightSpeed = mazeConsrains.GO_STRAIGHT_MAX_SPEED[deviceEnums.Side.RIGHT] + diff * mazeConsrains.P_GAIN 
            stm.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: int(leftSpeed), deviceEnums.Side.RIGHT: int(rightSpeed)}) #TODO: PD 制御にする?
    
    elif mazeConsrains.USE_MOVE_METHOD == mazeEnums.moveMethod.SEE_CORNER: # TODO: implement this method
        pass

    stm.sts3032.stop()
    detectWall(lidar, mapInstance)
    detectTileType(mapInstance)

def moveNextTile(direction: mazeEnums.absDirection, mapInstance: mazeMap.mazeMap,stm: stm.STM, lidar: ydlidar.CYdLidar) -> None:
    """
    @brief direction の方向のタイルへ一マス移動する
    @param direction: 移動方向
    @param mapInstance: 現在の迷路情報
    @param stm: 通信に使用する STM インスタンス
    @param lidar: 使用する LiDAR インスタンス
    """
    res = moveTile(direction, mapInstance, stm, lidar)
    mapInstance.moveTo(direction)
    return res

