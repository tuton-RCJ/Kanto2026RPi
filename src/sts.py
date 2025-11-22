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
    t = time.time()
    stmInstance.sts3032.turnLeft(50)
    while time.time() - t < deviceConstrains.TURN_SEC:
        pass
    stmInstance.sts3032.stop()

if __name__ == "__main__":
    main()
    