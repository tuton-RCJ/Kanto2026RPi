import os
import ydlidar
import numpy as np

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
        return scan.points
    else:
        raise Exception("Failed to get LiDAR scan")
    
def shutdownLidar(lidar: ydlidar.CYdLidar):
    lidar.turnOff()
    lidar.disconnecting()


def getCertainAngleDist(angle: int | list[int], points: list[ydlidar.LaserPoint]) -> int | dict[int]:
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
        closest_point = min(points, key=lambda p: abs(p.angle - a))
        distances.append(closest_point.range)

    return distances[0] if single else distances