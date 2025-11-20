import os
import ydlidar
import numpy as np
from dataclasses import dataclass

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
        for s in scan:
            res.append(Point(s.range,(s.angle - 180)%360))
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
        closest_point = min(points, key=lambda p: abs(p.angle - a))
        distances.append(closest_point.range)

    return distances[0] if single else distances

def getAbsAngle(nowDirection: int, angleRange: int, points: list[Point]) -> int:
    """Estimate which absolute NEWS direction the robot faces based on LiDAR data."""
    if angleRange <= 0:
        raise ValueError("angleRange must be positive")
    if not points:
        raise ValueError("points must not be empty")

    def normalizeDiff(target: float, values: np.ndarray) -> np.ndarray:
        """Return signed shortest angular difference between values and target."""
        return ((values - target + 540.0) % 360.0) - 180.0

    absAngles = np.array([(nowDirection + p.angle) % 360 for p in points], dtype=float)
    ranges = np.array([p.range for p in points], dtype=float)

    candidates = np.array([0.0, 90.0, 180.0, 270.0])
    bestAngle = None
    bestScore = np.inf
    for candidate in candidates:
        mask = np.abs(normalizeDiff(candidate, absAngles)) <= angleRange
        if not mask.any():
            continue
        # Median is robust against outliers and noisy returns.
        score = float(np.median(ranges[mask]))
        if score < bestScore:
            bestScore = score
            bestAngle = candidate
        elif score == bestScore:
            # Prefer the candidate closest to the current gyro heading to avoid jumps.
            currentDiff = abs(normalizeDiff(best_angle, np.array([nowDirection]))[0]) if best_angle is not None else np.inf
            newDiff = abs(normalizeDiff(candidate, np.array([nowDirection]))[0])
            if newDiff < currentDiff:
                best_angle = candidate

    if best_angle is None:
        raise RuntimeError("Unable to determine absolute angle from LiDAR points")
    return int(best_angle)
