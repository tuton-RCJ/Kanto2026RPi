from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConsrains
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
                    #print(mapInstance.renderKnownTileAndWall())
                nextDirection = mapInstance.getNearestUnexploredTile()

            returnPath = mapInstance.getPathTo((20, 20))
            print(f"Return Path: {returnPath}")
            
            if len(returnPath) > 0:
                for direction in returnPath:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    time.sleep(0.5) 
            else:
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