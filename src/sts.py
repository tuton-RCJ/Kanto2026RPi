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
    try:
        while True:
            stmInstance = stm.STM()
            stmInstance.update()
            stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: 0, deviceEnums.Side.RIGHT: 0})
            print(stmInstance.loadcell.pressed)
    except:
        import traceback
        traceback.print_exc()
        stmInstance.sts3032.stop()
if __name__ == "__main__":
    main()
    