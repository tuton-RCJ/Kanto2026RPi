<<<<<<< HEAD
import os
import math
import ydlidar
import numpy as np
from dataclasses import dataclass
from . import deviceConstrains
@dataclass
class Point:
    """
    @brief: LiDAR の点群データ構造体。
    @note range は cm, angle は相対角度、反時計回りに正。
    """
    range: int
    angle: int



def initializeLidar(port: str = "/dev/ttyAMA4", baudrate: int = 230400) -> ydlidar.CYdLidar:
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
            res.append(Point(s.range * 100,((s.angle - np.pi)%(np.pi*2)*360/(np.pi*2)+4)%360)) # LiDAR の角度補正 4 度
        return res
    else:
        raise Exception("Failed to get LiDAR scan")
    
def shutdownLidar(lidar: ydlidar.CYdLidar):
    lidar.turnOff()
    lidar.disconnecting()


def getCertainAngleDist(angle: int | list[int], points: list[Point]) -> int | dict[int]:
    """
    @brief 指定した角度の距離を取得する
    @param angle: 取得したい角度(度). 複数指定する場合はリストで渡す
    @param points: LiDAR のスキャンデータのリスト
    @return 指定した角度の距離(cm). 複数指定した場合はリストで返す
    @memo: 角度はロボット正面を 0 度として、反時計回りに増加する
    """

    if isinstance(angle, int):
        angle = [angle]
        single = True
    else:
        single = False

    distances = []
    for a in angle:
        dist = -1
        for p in points:
            if abs(p.angle - a) % 360 <= deviceConstrains.LiDAR_DIST_ANGLE_RANGE:
                if p.range == 0:
                    continue
                dist = max(dist, p.range)
        distances.append(dist)
    return distances[0] if single else distances

def getRelativeAngle(targetDirection: int, frontDirection: int, useDirection: int, angleRange: int, points: list[Point]) -> int:
    """
    @brief useDirection(相対角度) 方向の +-angleRange 内にある点群を用いて、ロボットが targetDirection 方向を向くための相対角度を計算する
    @param targetDirection: 目標の絶対角度(度)
    @param frontDirection: ロボットの正面が向いている絶対角度(度)
    @param useDirection: 相対角度を計算する基準方向(度)
    @param angleRange: nowDirection からの許容範囲(度)
    @param points: LiDAR のスキャンデータのリスト
    """

    if not points:
        return 0

    def normalize(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    usable_points = []
    for point in points:
        if point.range <= 0:
            continue
        relative = normalize(point.angle - useDirection)
        if abs(relative) <= angleRange:
            usable_points.append(point)

    if len(usable_points) < 2:
        return int(round(normalize(targetDirection - frontDirection)))

    xs = []
    ys = []
    for point in usable_points:
        rad = math.radians(point.angle)
        xs.append(point.range * math.cos(rad))
        ys.append(point.range * math.sin(rad))

    n = len(xs)
    sum_x = sum(xs)
    sum_y = sum(ys)
    sum_x2 = sum(x * x for x in xs)
    sum_xy = sum(x * y for x, y in zip(xs, ys))

    denom = n * sum_x2 - sum_x * sum_x
    if abs(denom) < 1e-6:
        alpha = math.copysign(float("inf"), sum_xy if sum_xy != 0 else 1.0)
    else:
        alpha = (n * sum_xy - sum_x * sum_y) / denom

    measured_relative = math.degrees(math.atan2(-1.0, alpha))

    measured_absolute = (frontDirection + measured_relative) % 360
    turn = normalize(targetDirection - measured_absolute)
    return int(round(turn))

def liDARShutdown(lidar: ydlidar.CYdLidar):
    """
    @brief LiDAR をシャットダウンする
    @param lidar: 使用する LiDAR インスタンス
    """
    lidar.turnOff()
    lidar.disconnecting()