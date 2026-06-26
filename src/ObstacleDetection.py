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
            # print(pts)
            # dist = LiDAR.getCertainAngleDist(0,pts)
            # for deg in [0,90,180,270]:
            #     isushape = LiDAR.detectUshapedTile_ROI(deg, pts)
            #     print(f"U-shaped tile detection at {deg} degrees: {isushape}")
            # print("---------")
            # for deg in [0,90,180,270]:
            #     dist = LiDAR.getCertainAngleDist(deg, pts)
            #     print(f"Distance at {deg} degrees: {dist} mm")
            #     # detection = LiDAR.judgeWallCertainAngle(0, pts)
            #     # print(detection)
            # LiDAR.exportPointCloudImg_Fast(pts)
            detection = LiDAR.judgeWallCertainAngle(0, pts)
            print(detection)
            # time.sleep(2)
    except KeyboardInterrupt:
        LiDAR.liDARShutdown(lidarInstance)

if __name__ == "__main__":
    main()
        