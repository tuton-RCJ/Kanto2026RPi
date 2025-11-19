from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConsrains
import ydlidar


def main():
    mapInstance = mazeMap.mazeMap()
    stmInstance = stm.STM()
    lidarInstance = ydlidar.CYdLidar()
    LiDAR.initializeLidar(lidarInstance)

    try:
        while True:
            points = lidarInstance.getLidarPoints()
            frontDist = LiDAR.getCertainAngleDist(0, points)
            print(f"Front Distance: {frontDist} cm")
    except KeyboardInterrupt:
        LiDAR.shutdownLidar(lidarInstance)
    """
    nextDirection = MapInstance.getNearestUnexploredTile()
    while nextDirection is not None:

        for direction in nextDirection:
            MoveTile.moveNextTile(direction, MapInstance, stmInstance, lidarInstance)
        nextDirection = MapInstance.getNearestUnexploredTile()

    returnPath = MapInstance.getPathTo((20, 20))

    if returnPath is not None:
        for direction in returnPath:
            MoveTile.moveNextTile(direction, MapInstance, stmInstance, lidarInstance)
    """
if __name__ == "__main__":
    main()