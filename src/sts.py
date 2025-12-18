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
    stmInstance = stm.STM()
    try:
        while True:
            stmInstance.update()
            print(stmInstance.loadcell.pressed)
            print(stmInstance.loadcell.getRawValue())
    except:
        stmInstance.sts3032.stop()
if __name__ == "__main__":
    main()
    