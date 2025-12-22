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
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    cs = colorsensor.ColorSensor()
    while True:
        stmInstance.update()
        cs.update()
        print(f"gyro Heading: {stmInstance.gyro.getValue().heading} deg, Pitch: {stmInstance.gyro.getValue().pitch} deg, Roll: {stmInstance.gyro.getValue().roll} deg")


if __name__ == "__main__":
    main()