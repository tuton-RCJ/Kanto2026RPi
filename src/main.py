from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    mapInstance = mazeMap.mazeMap()
    lidarInstance = LiDAR.initializeLidar()
    try:
        """
        stmInstance.update()
        print(f"tofDistance: {stmInstance.tof.getDistance()} cm")
        #print(f"AbsAngle: {stmInstance.gyro.getValue().heading}")
        """
        moveTile.detectWall(lidarInstance, mapInstance)
        tileType = moveTile.detectTileColor()
        mapInstance.setTileType(tileType)
        if mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN or mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
            time.sleep(0.1)
            t = time.time()
            leftFlag = True
            rightFlag = True
            while time.time() - t < 0.5:
                victimInfo = moveTile.getVictimInfo(stmInstance)
                if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] != vitimToWallType(victimInfo[deviceEnums.Side.LEFT]) and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN and leftFlag:
                    mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), moveTile.vitimToWallType(victimInfo[deviceEnums.Side.LEFT]))
                    moveTile.dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.LEFT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    leftFlag = False

                if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] != moveTile.vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]) and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN and rightFlag:
                    mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), moveTile.vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]))
                    moveTile.dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    rightFlag = False
            if leftFlag and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), mazeEnums.wallType.WALL)
            if rightFlag and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), mazeEnums.wallType.WALL)
        if mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 180) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN or mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
            moveTile.turnToCertainDirection((mapInstance.frontDirection.value + 90) % 360, stmInstance)
            mapInstance.frontDirection = mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)
            time.sleep(0.1)
            t = time.time()
            leftFlag = True
            rightFlag = True
            while time.time() - t < 0.5:                
                victimInfo = moveTile.getVictimInfo(stmInstance)
                if victimInfo[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] != moveTile.vitimToWallType(victimInfo[deviceEnums.Side.LEFT]) and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN and leftFlag:
                    mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), moveTile.vitimToWallType(victimInfo[deviceEnums.Side.LEFT]))
                    moveTile.dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.LEFT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    leftFlag = False

                if victimInfo[deviceEnums.Side.RIGHT] != deviceEnums.UnitVStatus.NOTHING and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] != moveTile.vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]) and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN and rightFlag:
                    mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), moveTile.vitimToWallType(victimInfo[deviceEnums.Side.RIGHT]))
                    moveTile.dropRescueKit(stmInstance, mapInstance, victimInfo, deviceEnums.Side.RIGHT)
                    print(f"Dropped rescue kit, detected victim info: {victimInfo}")
                    rightFlag = False

            if leftFlag and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 90) % 360), mazeEnums.wallType.WALL)
            if rightFlag and mapInstance.getWallType()[mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360)] == mazeEnums.wallType.WALL_BUT_NOSEEN:
                mapInstance.setWallType(mazeEnums.absDirection((mapInstance.frontDirection.value + 270) % 360), mazeEnums.wallType.WALL)

        print(mapInstance.wallTypes[20][20])
        nextDirection = mapInstance.getNearestUnexploredTile()
        print(f"Next Direction: {nextDirection}")
        while nextDirection is not None:
            for direction in nextDirection:
                moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                print(mapInstance.renderKnownTileAndWall())
                toggleswitchFlag = False
                stmInstance.update()
                if stmInstance.switch.getToggleSwitch1():
                    print("Exploration paused. Toggle switch 1 to resume.")
                while stmInstance.switch.getToggleSwitch1():
                    stmInstance.update()
                    toggleswitchFlag = True
                if toggleswitchFlag:
                    print("Exploration resumed.")
                    toggleswitchFlag = False
                    nowAngle = stmInstance.gyro.getValue().heading
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
                    time.sleep(1)  # Allow time for stabilization after resuming
                    break
            nextDirection = mapInstance.getNearestUnexploredTile()
                
        returnPath = mapInstance.getPathTo((20, 20))
        print(f"Return Path: {returnPath}")
        
        if len(returnPath) > 0:
            for direction in returnPath:
                moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                break
        print("Robot now at the starting position, Congratulations!")
        moveTile.flashLED(stmInstance, loopCount=5, intervalSec=0.5, color=[0, 255, 0])
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()
        exit(0)
    except:
        import traceback
        traceback.print_exc()
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()               
if __name__ == "__main__":
    main()