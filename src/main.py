from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    stmInstance.gyro.setHeadingOffset(stmInstance.gyro.getValue().heading)
    mapInstance = mazeMap.mazeMap()
    lidarInstance = LiDAR.initializeLidar()
    try:
        while True:
            """
            stmInstance.update()
            print(f"tofDistance: {stmInstance.tof.getDistance()} cm")
            #print(f"AbsAngle: {stmInstance.gyro.getValue().heading}")
            """
            moveTile.detectWall(lidarInstance, mapInstance)
            print(mapInstance.wallTypes[20][20])
            nextDirection = mapInstance.getNearestUnexploredTile()
            print(f"Next Direction: {nextDirection}")
            while nextDirection is not None:
                for direction in nextDirection:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    print(mapInstance.renderKnownTileAndWall())
                nextDirection = mapInstance.getNearestUnexploredTile()
                if stmInstance.switch.getToggleSwitch1():
                    print("Paused. Back to last silver tile")
                    mapInstance.loadCache()
                while stmInstance.switch.getToggleSwitch1():
                    time.sleep(0.1)
                    stmInstance.update()
                
                

            returnPath = mapInstance.getPathTo((20, 20))
            print(f"Return Path: {returnPath}")
            
            if len(returnPath) > 0:
                for direction in returnPath:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    time.sleep(0.5) 
                break
        print("Robot now at the starting position, Congratulations!")
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