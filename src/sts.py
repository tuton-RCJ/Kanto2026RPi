from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import deviceConstrains, deviceEnums
from mazeUtils import mazeEnums
from mazeUtils import mazeConsrains
import ydlidar
import time

def main():
    while True:
        stmInstance = stm.STM()
        stmInstance.update()
        print(stmInstance.loadcell.pressed)

if __name__ == "__main__":
    main()
    