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
    stmInstance.rescuekitservo.dropRescueKit(1, deviceEnums.Side.LEFT)
    while True:
        stmInstance.update()
        print(f"unitv: {stmInstance.unitv.getStatus()}")
        #print(f"AbsAngle: {stmInstance.gyro.getValue().heading}")
if __name__ == "__main__":
    main()