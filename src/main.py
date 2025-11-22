from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConsrains
import ydlidar
import time

def main():
    mapInstance = mazeMap.mazeMap()
    stmInstance = stm.STM()
    lidarInstance = LiDAR.initializeLidar()
    try:
        while True:
            """
            points = LiDAR.getLiDARScan(lidarInstance)
            frontDist = LiDAR.getCertainAngleDist(0, points)
            print(f"Front Distance: {frontDist} cm")
            angle = LiDAR.getRelativeAngle(
                mazeEnums.absDirection.NORTH.value,
                mapInstance.frontDirection.value,
                0,
                20,
                points,
            )
            print(f"Abs Angle on front deg: {min(angle, 360 - angle)} deg")
            """
            moveTile.detectWall(lidarInstance, mapInstance)
            print(mapInstance.wallTypes[20][20])
            nextDirection = mapInstance.getNearestUnexploredTile()
            print(f"Next Direction: {nextDirection}")
            while nextDirection is not None:
                for direction in nextDirection:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                nextDirection = mapInstance.getNearestUnexploredTile()

                returnPath = mapInstance.getPathTo((20, 20))
                print(f"Return Path: {returnPath}")
                
                if returnPath is not None:
                    for direction in returnPath:
                        moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
    except:
        import traceback
        traceback.print_exc()
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()               
if __name__ == "__main__":
    main()