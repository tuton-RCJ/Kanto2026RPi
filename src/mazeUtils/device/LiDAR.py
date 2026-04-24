import os
import ydlidar
import numpy as np
from dataclasses import dataclass
from . import deviceConstraints
from . import deviceEnums
@dataclass
class Point:
    """
    @brief: LiDAR の点群データ構造体。
    @note range は cm, angle は相対角度、反時計回りに正。
    """
    range: int
    angle: int

def regulationAngle(angle: int) -> int:
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle



def initializeLidar(port: str = "/dev/ttyAMA2", baudrate: int = 230400) -> ydlidar.CYdLidar:
    ydlidar.os_init()
    print("Available ports:", *ydlidar.lidarPortList())
    lidar = ydlidar.CYdLidar()
    lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
    lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
    lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    lidar.setlidaropt(ydlidar.LidarPropSampleRate, 4)
    lidar.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    lidar.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
    lidar.setlidaropt(ydlidar.LidarPropIntenstiy, True)
    if not lidar.initialize():
        raise Exception("Failed to initialize LiDAR")
    lidar.turnOn()
    return lidar

def getLiDARScan(lidar: ydlidar.CYdLidar) -> list[ydlidar.LaserPoint]:
    """
    @brief LiDAR のスキャンデータを取得する
    @param lidar: 使用する LiDAR インスタンス
    @return スキャンデータのリスト. ydlidar.LaserPoint.angle に相対角度(度), ydlidar.LaserPoint.range に距離(cm)が格納されている
    """
    scan = ydlidar.LaserScan()
    if lidar.doProcessSimple(scan):
        res = []
        for s in scan.points:
            res.append(Point(s.range * 100,(-((s.angle - np.pi/2)%(np.pi*2)*360/(np.pi*2)+4))%360)) # LiDAR の角度補正 4 度
        return res
    else:
        raise Exception("Failed to get LiDAR scan")

def getCertainAngleDist(angle: int | list[int], points: list[Point]) -> int | list[int]:
    """
    @brief 指定した角度の距離を取得する
    @param angle: 取得したい角度(度). 複数指定する場合はリストで渡す
    @param points: LiDAR のスキャンデータのリスト
    @return 指定した角度の距離(cm). 複数指定した場合はリストで返す
    @memo: 角度はロボット正面を 0 度として、反時計回りに増加する
    """

    if isinstance(angle, int | float):
        angle = [angle]
        single = True
    else:
        single = False

    distances = []
    for a in angle:
        minerror = 1e9
        dist = -1
        distlist = []
        for p in points:
            if p.range < 10 or p.range > 400:
                continue
            if abs(regulationAngle(p.angle - a)) < 20:
                distlist.append(p.range * np.cos(np.deg2rad(regulationAngle(p.angle - a))))
        if len(distlist) > 0:
            distlist.sort()
            distances.append(distlist[-1])
        else:
            distances.append(10**9)  # LiDAR の測定範囲外は非常に大きな値とする
    return distances[0] if single else distances

def isWallAheadTile(points: list[Point], side: deviceEnums.Side) -> bool:
    """
    @brief 指定したサイドの前方に壁があるかどうかを判定する
    @param points: LiDAR のスキャンデータのリスト
    @param side: 判定するサイド
    @return 壁がある場合は True, ない場合は False
    """
    res = True
    for p in points:
        if side == deviceEnums.Side.LEFT:
            if p.angle >= 10 and p.angle <= 90:
                if p.range*np.cos(np.deg2rad(p.angle-90)) > deviceConstraints.WALL_DETECTION_THRESHOLD_CM:
                    return False
        elif side == deviceEnums.Side.RIGHT:
            if p.angle >= 270 and p.angle <= 350:
                if p.range*np.cos(np.deg2rad(p.angle-270)) > deviceConstraints.WALL_DETECTION_THRESHOLD_CM:
                    return False
    return res

def liDARShutdown(lidar: ydlidar.CYdLidar):
    """
    @brief LiDAR をシャットダウンする
    @param lidar: 使用する LiDAR インスタンス
    """
    lidar.turnOff()
    lidar.disconnecting()