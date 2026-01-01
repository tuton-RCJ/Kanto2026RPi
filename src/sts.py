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
    while True:
        stmInstance.update()
        print(f"loadCellVoltage: {stmInstance.loadcell.getRawValue()} mV")
if __name__ == "__main__":
    main()