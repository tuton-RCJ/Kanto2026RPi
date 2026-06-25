import os
import ydlidar
import numpy as np
from dataclasses import dataclass
from config import get_logger
from . import deviceConstraints
from . import deviceEnums
from typing import overload
import math
import matplotlib.pyplot as plt

logger = get_logger(__name__)


@dataclass
class Point:
    """
    @brief: LiDAR の点群データ構造体。
    @note range は cm, angle は相対角度、反時計回りに正。
    """

    range: int
    angle: int


def regulationAngle(angle: int | float) -> int | float:
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle


def initializeLidar(
    port: str = "/dev/ttyAMA2", baudrate: int = 230400
) -> ydlidar.CYdLidar:
    ydlidar.os_init()
    logger.info(f"Available ports: {ydlidar.lidarPortList()}")
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


def getLiDARScan(lidar: ydlidar.CYdLidar) -> list[Point]:
    """
    @brief LiDAR のスキャンデータを取得する
    @param lidar: 使用する LiDAR インスタンス
    @return スキャンデータのリスト. ydlidar.LaserPoint.angle に相対角度(度), ydlidar.LaserPoint.range に距離(cm)が格納されている
    """
    scan = ydlidar.LaserScan()
    if lidar.doProcessSimple(scan):
        res = []
        for s in scan.points:
            res.append(
                Point(
                    s.range * 100,
                    (-((s.angle - np.pi / 2) % (np.pi * 2) * 360 / (np.pi * 2) + 4))
                    % 360,
                )
            )  # LiDAR の角度補正 4 度
        return res
    else:
        raise Exception("Failed to get LiDAR scan")


@overload
def getCertainAngleDist(angle: int | float, points: list[Point]) -> int: ...


@overload
def getCertainAngleDist(
    angle: list[int] | list[float], points: list[Point]
) -> list[int]: ...


def getCertainAngleDist(
    angle: int | float | list[int] | list[float], points: list[Point]
) -> int | list[int]:
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
                distlist.append(
                    p.range * np.cos(np.deg2rad(regulationAngle(p.angle - a)))
                )
        if len(distlist) > 0:
            distlist.sort()
            distances.append(distlist[-1])
        else:
            distances.append(10**9)  # LiDAR の測定範囲外は非常に大きな値とする
    return distances[0] if single else distances

def getCertainAngleDist_SinglePoint(angle: int, points: list[Point]) -> int:
    """
    @brief 指定した角度の距離を取得する(単一の点のみ)
    @param angle: 取得したい角度(度)
    @param points: LiDAR のスキャンデータのリスト
    @return 指定した角度の距離(cm)
    @memo: 角度はロボット正面を 0 度として、反時計回りに増加する
    """
    minerror = 1e9
    dist = -1
    for p in points:
        if p.range < 10 or p.range > 400:
            continue
        if abs(regulationAngle(p.angle - angle)) < 20:
            error = abs(regulationAngle(p.angle - angle))
            if error < minerror:
                minerror = error
                dist = p.range * np.cos(np.deg2rad(regulationAngle(p.angle - angle)))
    return dist

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
                if (
                    p.range * np.cos(np.deg2rad(p.angle - 90))
                    > deviceConstraints.WALL_DETECTION_THRESHOLD_CM
                ):
                    return False
        elif side == deviceEnums.Side.RIGHT:
            if p.angle >= 270 and p.angle <= 350:
                if (
                    p.range * np.cos(np.deg2rad(p.angle - 270))
                    > deviceConstraints.WALL_DETECTION_THRESHOLD_CM
                ):
                    return False
    return res


def judgeWallCertainAngle(angle: int, points: list[Point] ) -> deviceEnums.judgeWallResult:
    """
    @brief 指定した角度について、中央障害物・左障害物・右障害物・壁・壁なしのいずれかを判定する
    @param points: LiDAR のスキャンデータのリスト
    @param angle: 判定する角度(度)
    @return 壁なし: 0, 壁: 1, 中央障害物: 2, 左障害物: 3, 右障害物: 4
    """
    center_dist = getCertainAngleDist_SinglePoint(angle, points)

    # 連結成分を判定
    # angleから±45度の範囲を探索し、連結成分の長さを計算する

    pre_left_point = Point(range=center_dist, angle=angle)
    pre_right_point = Point(range=center_dist, angle=angle)
    continue_left = True
    continue_right = True
    connected_component_length = 0  # 連結成分の長さ(cm) 曲線の長さ
    for dp in range(0, 46, 2):
        left_point = Point(
            range=getCertainAngleDist_SinglePoint(angle - dp, points), angle=angle - dp
        )
        right_point = Point(
            range=getCertainAngleDist_SinglePoint(angle + dp, points), angle=angle + dp
        )
        if abs(
            left_point.range - pre_left_point.range
        ) > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM or (not continue_left):
            continue_left = False
        else:
            connected_component_length += math.sqrt(
                pre_left_point.range**2
                + left_point.range**2
                - 2
                * pre_left_point.range
                * left_point.range
                * math.cos(math.radians(left_point.angle - pre_left_point.angle))
            )
            pre_left_point = left_point

        if abs(
            right_point.range - pre_right_point.range
        ) > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM or (not continue_right):
            continue_right = False
        else:
            connected_component_length += math.sqrt(
                pre_right_point.range**2
                + right_point.range**2
                - 2
                * pre_right_point.range
                * right_point.range
                * math.cos(math.radians(right_point.angle - pre_right_point.angle))
            )
            pre_right_point = right_point

    connected_component_length_horizontal = abs(
        pre_right_point.range * np.sin(np.deg2rad(pre_right_point.angle - angle))
    ) + abs(pre_left_point.range * np.sin(np.deg2rad(pre_left_point.angle - angle))) # 連結成分の長さ(cm) 見ている方向に垂直な成分を見る

    # print(f"center_dist: {center_dist}, connected_component_length_horizontal: {connected_component_length_horizontal}")

    if (
        center_dist < deviceConstraints.WALL_DETECTION_THRESHOLD_CM
        and connected_component_length_horizontal > deviceConstraints.WALL_DETECTION_CONNECTED_COMPONENT_THRESHOLD_CM
    ):
        return deviceEnums.judgeWallResult.WALL  # 壁
    elif (
        center_dist < deviceConstraints.WALL_DETECTION_THRESHOLD_CM
        and connected_component_length_horizontal <= deviceConstraints.WALL_DETECTION_CONNECTED_COMPONENT_THRESHOLD_CM
    ):
        return deviceEnums.judgeWallResult.CENTER_OBSTACLE  # 中央障害物

    obstacle_left_flag = False
    obstacle_right_flag = False
    for dp in range(0, 40, 3):
        left_point = Point(
            range=getCertainAngleDist_SinglePoint(angle + dp, points), angle=angle + dp
        )
        right_point = Point(
            range=getCertainAngleDist_SinglePoint(angle - dp, points), angle=angle - dp
        )

        if left_point.range < deviceConstraints.SIDE_OBSTACLE_DETECTION_THRESHOLD_CM and abs(left_point.range * np.sin(np.deg2rad(dp))) < deviceConstraints.SIDE_OBSTACLE_DETECTION_X_LIMIT_CM:
            obstacle_left_flag = True
        if (
            right_point.range < deviceConstraints.SIDE_OBSTACLE_DETECTION_THRESHOLD_CM
            and abs(right_point.range * np.sin(np.deg2rad(dp))) < deviceConstraints.SIDE_OBSTACLE_DETECTION_X_LIMIT_CM
        ):
            obstacle_right_flag = True

    if obstacle_left_flag and not obstacle_right_flag:
        return deviceEnums.judgeWallResult.LEFT_OBSTACLE  # 左障害物
    elif not obstacle_left_flag and obstacle_right_flag:
        return deviceEnums.judgeWallResult.RIGHT_OBSTACLE  # 右障害物
    elif obstacle_left_flag and obstacle_right_flag:
        logger.warning(
            "Both left and right obstacles detected, returning central obstacle"
        )
        return deviceEnums.judgeWallResult.CENTER_OBSTACLE  # 中央障害物
    else:
        return deviceEnums.judgeWallResult.NO_WALL  # 壁なし


def liDARShutdown(lidar: ydlidar.CYdLidar):
    """
    @brief LiDAR をシャットダウンする
    @param lidar: 使用する LiDAR インスタンス
    """
    lidar.turnOff()
    lidar.disconnecting()


def exportPointCloudImg(points: list[Point], output_path: str = "map.png"):
    """LiDARの点群データからマップ画像を生成し保存する。

    Args:
        points (list[Point]): 点群データのリスト
        output_path (str): 出力する画像ファイルのパス
    """
    x_coords = []
    y_coords = []

    for p in points:
        angle_rad = math.radians(p.angle)

        # 極座標 (r, theta) から 直交座標 (x, y) への変換
        x = p.range * math.cos(angle_rad)
        y = p.range * math.sin(angle_rad)

        x_coords.append(x)
        y_coords.append(y)

    # グラフのプロット設定
    plt.figure(figsize=(10, 10))  # 10x10インチのサイズ

    # 点群をプロット（散布図: scatter）
    # sは点のサイズ、cは色、alphaは透明度
    plt.scatter(x_coords, y_coords, c="blue", s=2, alpha=0.6, label="Points")

    # LiDARの基準位置 (0, 0) を赤色のXでプロット
    plt.scatter(0, 0, c="red", marker="x", s=100, label="LiDAR Origin")

    # グラフの見た目の調整
    plt.title("LiDAR Point Cloud Map")
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.grid(True, linestyle="--", alpha=0.5)  # グリッド線の表示
    plt.axis("equal")  # X軸とY軸のスケール（比率）を1:1にする
    plt.legend(loc="upper right")

    # 画像として保存 (dpiを高めに設定して鮮明に)
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()  # メモリ解放のためにクローズ