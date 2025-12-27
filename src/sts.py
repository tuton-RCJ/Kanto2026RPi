from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    LiDARInstance = LiDAR.initializeLidar()
    while True:
        stmInstance.update()
        print(f"tofDistance: {stmInstance.tof.getDistance()} cm")
if __name__ == "__main__":
    main()