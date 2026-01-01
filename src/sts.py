from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    LiDARInstance = LiDAR.initializeLidar()
    colorsensorInstance = colorsensor.ColorSensor()
    
    while True:
        stmInstance.update()
        print(f"gyro value: {stmInstance.gyro.getValue()}")
if __name__ == "__main__":
    main()