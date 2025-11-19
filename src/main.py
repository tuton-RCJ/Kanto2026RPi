from mazeUtils import mazeMap
from mazeUtils import MoveTile
from mazeUtils.device import stm
from mazeUtils import mazeEnums
from mazeUtils import mazeConsrains
import ydlidar

def main():
    MapInstance = mazeMap.mazeMap()
    stmInstance = stm.STM()
    lidarInstance = ydlidar.Cydliar()
    nextDirection = MapInstance.getNearestUnexploredTile()
    while nextDirection is not None:

        for direction in nextDirection:
            MoveTile.moveNextTile(direction, MapInstance, stmInstance, lidarInstance)
        nextDirection = MapInstance.getNearestUnexploredTile()

    returnPath = MapInstance.getPathTo((20, 20))

    if returnPath is not None:
        for direction in returnPath:
            MoveTile.moveNextTile(direction, MapInstance, stmInstance, lidarInstance)

if __name__ == "__main__":
    main()