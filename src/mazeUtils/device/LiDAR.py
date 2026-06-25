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

# @dataclass
# class Point:
#     """
#     @brief: LiDAR の点群データ構造体。
#     @note range は cm, angle は相対角度、反時計回りに正。
#     """

#     range: int
#     angle: int


@dataclass
class ScanMap:
    """高速化のための事前計算済みLiDARデータ構造体"""
    ranges: np.ndarray  # 長さ360の配列 (indexが角度、値が距離)
    x: np.ndarray       # 長さ360の配列 (indexが角度、値がX座標)
    y: np.ndarray       # 長さ360の配列 (indexが角度、値がY座標)
    valid: np.ndarray   # 長さ360のboolean配列 (有効な点かどうかのフラグ)

def processLiDARScanToMap(scan_points: list[tuple[int, int]]) -> ScanMap:
    """
    スキャン生データを360度のルックアップテーブルに変換する前処理 (O(N))
    1スキャンにつき1回だけ呼ぶ。
    """
    # 360度分の配列を初期化 (初期値は巨大な数)
    ranges = np.full(360, 10**9, dtype=np.float32)
    
    # ydlidarからの生データを取り出すと仮定
    for s in scan_points:
        # 距離のフィルタリング (10cm未満、400cm以上、または0.0は除外)
        r_cm = s[0] * 100
        if r_cm < 10 or r_cm > 400 or r_cm == 0.0:
            continue
            
        # 角度計算 (0~359の整数に丸める
        idx = int(round(s[1])) % 360
        
        # 同じ角度に複数の点がある場合は、近い方を採用する
        if r_cm < ranges[idx]:
            ranges[idx] = r_cm

    # 有効な点のフラグ
    valid = ranges < 400

    # 角度の配列 (0~359度) をラジアンに変換して一括でX, Yを計算
    rads = np.deg2rad(np.arange(360))
    x = np.where(valid, ranges * np.cos(rads), 0.0)
    y = np.where(valid, ranges * np.sin(rads), 0.0)

    return ScanMap(ranges=ranges, x=x, y=y, valid=valid)


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


def getLiDARScan(lidar: ydlidar.CYdLidar) -> ScanMap:
    """
    @brief LiDAR のスキャンデータを取得する
    @param lidar: 使用する LiDAR インスタンス
    @return スキャンデータのリスト. ydlidar.LaserPoint.angle に相対角度(度), ydlidar.LaserPoint.range に距離(cm)が格納されている
    """
    scan = ydlidar.LaserScan()
    if lidar.doProcessSimple(scan):
        res: list[tuple[int, int]] = []
        for s in scan.points:
            res.append(
                (
                    s.range * 100,
                    (-((s.angle - np.pi / 2) % (np.pi * 2) * 360 / (np.pi * 2) + 4))
                    % 360
                )
            )  # LiDAR の角度補正 4 度
        return processLiDARScanToMap(res)  # 前処理して返す
    else:
        raise Exception("Failed to get LiDAR scan")


@overload
def getCertainAngleDist(angle: int | float, points: ScanMap) -> int: ...


@overload
def getCertainAngleDist(
    angle: list[int] | list[float], points: ScanMap
) -> list[int]: ...


def getCertainAngleDist(
    angle: int | float | list[int] | list[float], points: ScanMap
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
        target_deg = int(round(a)) % 360
        idx_range = np.mod(target_deg + np.arange(-20, 21), 360)  # ±20度の範囲を探索
        
        valid_mask = points.valid[idx_range]
        if not np.any(valid_mask):
            distances.append(10**9)  # LiDAR の測定範囲外は非常に大きな値とする
            continue
        
        valid_indices = idx_range[valid_mask]
        
        target_rad = math.radians(a)
        dir_x = math.cos(target_rad)
        dir_y = math.sin(target_rad)
        
        x_vals = points.x[valid_indices]
        y_vals = points.y[valid_indices]
        
        proj_dists = x_vals * dir_x + y_vals * dir_y
        
        distances.append(np.max(proj_dists) if len(proj_dists) > 0 else 10**9)
        
    return distances[0] if single else distances

def getCertainAngleDist_SinglePoint(angle: int, points: ScanMap) -> int:
    """
    @brief 指定した角度の距離を取得する(単一の点のみ)
    @param angle: 取得したい角度(度)
    @param points: LiDAR のスキャンデータのリスト
    @return 指定した角度の距離(cm)
    @memo: 角度はロボット正面を 0 度として、反時計回りに増加する
    """
    target_deg = int(round(angle)) % 360
    dist = points.ranges[target_deg]
    return dist

def isWallAheadTile(points: ScanMap, side: deviceEnums.Side) -> bool:
    """
    @brief 指定したサイドの前方に壁があるかどうかを判定する
    @param points: LiDAR のスキャンデータのリスト
    @param side: 判定するサイド
    @return 壁がある場合は True, ない場合は False
    """

    if side == deviceEnums.Side.LEFT:
        valid_mask = points.valid[10:91]
        y_vals = points.y[10:91][valid_mask]
        
        if np.any(y_vals > deviceConstraints.WALL_DETECTION_THRESHOLD_CM):
            return False
    elif side == deviceEnums.Side.RIGHT:
        valid_mask = points.valid[270:351]
        y_vals = points.y[270:351][valid_mask]
        
        if np.any(y_vals < -deviceConstraints.WALL_DETECTION_THRESHOLD_CM):
            return False
    return True


def judgeWallCertainAngle(angle: int, points: ScanMap ) -> deviceEnums.judgeWallResult:
    """
    @brief 指定した角度について、中央障害物・左障害物・右障害物・壁・壁なしのいずれかを判定する
    @param points: LiDAR のスキャンデータのリスト
    @param angle: 判定する角度(度)
    @return 壁なし: 0, 壁: 1, 中央障害物: 2, 左障害物: 3, 右障害物: 4
    """
    center_dist = ScanMap.ranges[angle]

    center_rad = math.radians(angle)
    dir_x = math.cos(center_rad)
    dir_y = math.sin(center_rad)
    

    # 連結成分を判定
    # angleから±45度の範囲を探索し、連結成分の長さを計算する
    pre_left_idx = angle
    pre_right_idx = angle
    continue_left = True
    continue_right = True
    connected_component_length = 0  # 連結成分の長さ(cm) 曲線の長さ
    
    
    for dp in range(0, 46, 2):
        l_idx = (angle + dp) % 360
        r_idx = (angle - dp) % 360
        
        if continue_left and points.valid[l_idx] and points.valid[pre_left_idx]:
            dist_diff = math.hypot(
                points.x[l_idx] - points.x[pre_left_idx],
                points.y[l_idx] - points.y[pre_left_idx]
            )
            
            if abs(points.ranges[l_idx] - points.ranges[pre_left_idx]) > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM:
                continue_left = False
            else:
                connected_component_length += dist_diff
                pre_left_idx = l_idx
        else:
            continue_left = False
        
        if continue_right and points.valid[r_idx] and points.valid[pre_right_idx]:
            dist_diff = math.hypot(
                points.x[r_idx] - points.x[pre_right_idx],
                points.y[r_idx] - points.y[pre_right_idx]
            )
            
            if abs(points.ranges[r_idx] - points.ranges[pre_right_idx]) > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM:
                continue_right = False
            else:
                connected_component_length += dist_diff
                pre_right_idx = r_idx
        else:
            continue_right = False
            
    left_cross = abs(points.x[pre_left_idx] * dir_y - points.y[pre_left_idx] * dir_x)
    right_cross = abs(points.x[pre_right_idx] * dir_y - points.y[pre_right_idx] * dir_x)
    connected_component_length_horizontal = left_cross + right_cross  # 連結成分の長さ(cm) 見ている方向に垂直な成分を見る
    
    # print(f"center_dist: {center_dist}, connected_component_length_horizontal: {connected_component_length_horizontal}")

    if center_dist < deviceConstraints.WALL_DETECTION_THRESHOLD_CM:
        if connected_component_length_horizontal > deviceConstraints.WALL_DETECTION_CONNECTED_COMPONENT_THRESHOLD_CM:
            return deviceEnums.judgeWallResult.WALL  # 壁
        else:
            return deviceEnums.judgeWallResult.CENTER_OBSTACLE  # 中央障害物
        
    #### 左右障害物の判定
    
    obstacle_left_flag = False
    obstacle_right_flag = False
    
    for dp in range(0, 40, 3):
        l_idx = (angle + dp) % 360
        r_idx = (angle - dp) % 360
        
        if points.valid[l_idx]:
            l_range = points.ranges[l_idx]
            l_y_dist = abs(points.x[l_idx] * dir_y - points.y[l_idx] * dir_x)
            if l_range < deviceConstraints.SIDE_OBSTACLE_DETECTION_THRESHOLD_CM and l_y_dist < deviceConstraints.SIDE_OBSTACLE_DETECTION_X_LIMIT_CM:
                obstacle_left_flag = True
        
        if points.valid[r_idx]:
            r_range = points.ranges[r_idx]
            r_y_dist = abs(points.x[r_idx] * dir_y - points.y[r_idx] * dir_x)
            if r_range < deviceConstraints.SIDE_OBSTACLE_DETECTION_THRESHOLD_CM and r_y_dist < deviceConstraints.SIDE_OBSTACLE_DETECTION_X_LIMIT_CM:
                obstacle_right_flag = True
            
        if obstacle_left_flag and obstacle_right_flag:
            break  # 両方の障害物が検出されたらループを抜ける
        
    if obstacle_left_flag and not obstacle_right_flag:
        return deviceEnums.judgeWallResult.LEFT_OBSTACLE  # 左障害物
    elif not obstacle_left_flag and obstacle_right_flag:
        return deviceEnums.judgeWallResult.RIGHT_OBSTACLE  # 右障害物
    elif obstacle_left_flag and obstacle_right_flag:
        logger.warning(
            "Both left and right obstacles detected, returning LEFT_OBSTACLE as default."
        )
        return deviceEnums.judgeWallResult.LEFT_OBSTACLE  # 左障害物
    else:
        return deviceEnums.judgeWallResult.NO_WALL  # 壁なし


def liDARShutdown(lidar: ydlidar.CYdLidar):
    """
    @brief LiDAR をシャットダウンする
    @param lidar: 使用する LiDAR インスタンス
    """
    lidar.turnOff()
    lidar.disconnecting()


def exportPointCloudImg_Fast(scan_map: 'ScanMap', output_path: str = "map.png"):
    """LiDARの点群データからマップ画像を生成し保存する。

    Args:
        scan_map (ScanMap): 前処理済みのLiDARデータマップ
        output_path (str): 出力する画像ファイルのパス
    """
    # 有効な点（validがTrueのインデックス）のX, Y座標を一括抽出
    x_coords = scan_map.x[scan_map.valid]
    y_coords = scan_map.y[scan_map.valid]

    # グラフのプロット設定
    plt.figure(figsize=(10, 10))  # 10x10インチのサイズ

    # 点群をプロット（散布図: scatter）
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