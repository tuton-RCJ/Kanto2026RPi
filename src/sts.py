from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils.device import camera
from mazeUtils.device import photoReflector
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    lidarInstance = LiDAR.initializeLidar()   
    while True:
        print("Left:",LiDAR.isWallAheadTile(LiDAR.getLiDARScan(lidarInstance), deviceEnums.Side.LEFT))
        print("Right:",LiDAR.isWallAheadTile(LiDAR.getLiDARScan(lidarInstance), deviceEnums.Side.RIGHT))
        time.sleep(1)

if __name__ == "__main__":
    main()