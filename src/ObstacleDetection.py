from config import setup_logging, get_logger
from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
import ydlidar
import time

def main():
    lidarInstance = LiDAR.initializeLidar()
    try:
        while True:
            pts = LiDAR.getLiDARScan(lidarInstance)
            # dist = LiDAR.getCertainAngleDist(0,pts)
            detection = LiDAR.judgeWallCertainAngle(0, pts)
            print(detection)
            # LiDAR.exportPointCloudImg(pts)
    except KeyboardInterrupt:
        LiDAR.liDARShutdown(lidarInstance)

if __name__ == "__main__":
    main()
        