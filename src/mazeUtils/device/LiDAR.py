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

pre_scan_map: ScanMap | None = None  # 前回のLiDARデータマップを保持するグローバル変数

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
        r_cm = s[0]
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
        _scanMap = processLiDARScanToMap(res) 
        global pre_scan_map
        pre_scan_map = _scanMap  # グローバル変数に保存
        return _scanMap
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
        _detect_range = (-10, 21) if target_deg==90 else ( (-20, 11) if target_deg==270 else (-20, 21)) # 横をみるときは前側の範囲狭める
        idx_range = np.mod(target_deg + np.arange(*_detect_range), 360)  # _detect_rangeの範囲を探索
        
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
    center_dist = points.ranges[angle]

    center_rad = math.radians(angle)
    dir_x = math.cos(center_rad)
    dir_y = math.sin(center_rad)
    
    # LiDARの正面±ANGLE_LIMITの範囲で中央障害物の測定開始点を探索
    start_angle = angle
    for dp in range(0, deviceConstraints.CENTER_OBSTACLE_DETECTION_ANGLE_LIMIT_DEG + 1, 1):
        l_idx = (start_angle + dp) % 360
        r_idx = (start_angle - dp) % 360
        
        if points.valid[l_idx]:
            l_range = points.ranges[l_idx]
            l_y_dist = abs(points.x[l_idx] * dir_y - points.y[l_idx] * dir_x)
            if l_range < deviceConstraints.WALL_DETECTION_THRESHOLD_CM and l_y_dist < deviceConstraints.CENTER_OBSTACLE_DETECTION_X_LIMIT_CM:
                center_dist = l_range
                start_angle = l_idx
                break
        if points.valid[r_idx]:
            r_range = points.ranges[r_idx]
            r_y_dist = abs(points.x[r_idx] * dir_y - points.y[r_idx] * dir_x)
            if r_range < deviceConstraints.WALL_DETECTION_THRESHOLD_CM and r_y_dist < deviceConstraints.CENTER_OBSTACLE_DETECTION_X_LIMIT_CM:
                center_dist = r_range
                start_angle = r_idx
                break

    

    # 連結成分を判定
    # angleから±45度の範囲を探索し、連結成分の長さを計算する
    pre_left_idx = start_angle
    pre_right_idx = start_angle
    continue_left = True
    continue_right = True
    connected_component_length = 0  # 連結成分の長さ(cm) 曲線の長さ
    connected_component_length_horizontal = 0   # 連結成分の長さ(cm) 見ている方向に垂直な成分を見る
    
    
    for dp in range(0, 46, 1):
        l_idx = (start_angle + dp) % 360
        r_idx = (start_angle - dp) % 360
        
        if continue_left and points.valid[l_idx] and points.valid[pre_left_idx]:
            dist_diff = math.hypot(
                points.x[l_idx] - points.x[pre_left_idx],
                points.y[l_idx] - points.y[pre_left_idx]
            )
            
            if abs(points.ranges[l_idx] - points.ranges[pre_left_idx]) > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM:
                continue_left = False
            else:
                connected_component_length += dist_diff
                connected_component_length_horizontal += abs(abs(points.x[l_idx] * dir_y - points.y[l_idx] * dir_x) - abs(points.x[pre_left_idx] * dir_y - points.y[pre_left_idx] * dir_x))
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
                connected_component_length_horizontal += abs(abs(points.x[r_idx] * dir_y - points.y[r_idx] * dir_x) - abs(points.x[pre_right_idx] * dir_y - points.y[pre_right_idx] * dir_x))
                pre_right_idx = r_idx
        else:
            continue_right = False
            
    
    print(f"angle: {angle}, center_dist: {center_dist}, connected_component_length_horizontal: {connected_component_length_horizontal}")

    if center_dist < deviceConstraints.WALL_DETECTION_THRESHOLD_CM if (angle==0 or angle == 180) else deviceConstraints.WALL_DETECTION_THRESHOLD_CM_SIDE:
        if connected_component_length_horizontal > deviceConstraints.WALL_DETECTION_CONNECTED_COMPONENT_THRESHOLD_CM:
            return deviceEnums.judgeWallResult.WALL  # 壁
        else:
            if center_dist < 15 and getCertainAngleDist(angle,points) < 28:
                return deviceEnums.judgeWallResult.WALL  # あまりに近いと視野に入りきらなくて中央障害物と判断してしまうけど、これは壁としよう。
            return deviceEnums.judgeWallResult.CENTER_OBSTACLE  # 中央障害物
        
    #### 左右障害物の判定
    
    obstacle_left_flag = False
    obstacle_right_flag = False
    
    for dp in range(0, 40, 2):
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
            "Both left and right obstacles detected, returning WALL as default."
        )
        return deviceEnums.judgeWallResult.WALL  # 左障害物
    else:
        return deviceEnums.judgeWallResult.NO_WALL  # 壁なし


def liDARShutdown(lidar: ydlidar.CYdLidar):
    """
    @brief LiDAR をシャットダウンする
    @param lidar: 使用する LiDAR インスタンス
    """
    lidar.turnOff()
    lidar.disconnecting()

def detectUshapedTile_conn(points: ScanMap, angle: int) -> bool:
    """
    @brief コの字型（3辺が壁で囲まれている形状）のマスを検出する
    @param points: LiDAR のスキャンデータのリスト
    @param angle: 探す方向の角度
    @return U字型のタイルが検出された場合は True, それ以外は False
    """
    
    """
    方針
    1. 指定した角度の正面に壁があるかを判定する
    2. 正面から±40°の範囲で、正面の点と連続している点を追っていく
    3. 左右の端点がともに、正面方向と平行な方向と垂直な平行のしきい値を満たしている時、コの字と判定
    
    """
    
    dir_rad = math.radians(angle)
    dir_x = math.cos(dir_rad)
    dir_y = math.sin(dir_rad)
    
    center_dist = points.ranges[angle]
    
    if not (30< center_dist < 60):
        return False  # 正面壁が条件を満たさない
    
    pre_left_idx = angle
    pre_right_idx = angle
    continue_left = True
    continue_right = True
    
    left_condition_met = False
    right_condition_met = False
    
    for dp in range(0, 41, 1):
        l_idx = (angle + dp) % 360
        r_idx = (angle - dp) % 360
        
        if continue_left and points.valid[l_idx] and points.valid[pre_left_idx]:
            dist_diff = math.hypot(
                points.x[l_idx] - points.x[pre_left_idx],
                points.y[l_idx] - points.y[pre_left_idx]
            )
            
            if dist_diff > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM:
                continue_left = False
            else:
                pre_left_idx = l_idx
                
                # 左の端点が、正面方向と平行な方向と垂直な平行のしきい値を満たしているかを判定
                left_y_dist = abs(points.x[l_idx] * dir_y - points.y[l_idx] * dir_x)
                left_x_dist = abs(points.x[l_idx] * dir_x + points.y[l_idx] * dir_y)
                if 0 < left_y_dist < 30 and 0 < left_x_dist < 20:
                    left_condition_met = True
                    continue_left = False  # 条件を満たしたら探索終了
        else:
            continue_left = False
        
        if continue_right and points.valid[r_idx] and points.valid[pre_right_idx]:
            dist_diff = math.hypot(
                points.x[r_idx] - points.x[pre_right_idx],
                points.y[r_idx] - points.y[pre_right_idx]
            )
            
            if dist_diff > deviceConstraints.CONNECTED_COMPONENT_THRESHOLD_CM:
                continue_right = False
            else:
                pre_right_idx = r_idx
                # 右の端点が、正面方向と平行な方向と垂直な平行のしきい値を満たしているかを判定
                right_y_dist = abs(points.x[r_idx] * dir_y - points.y[r_idx] * dir_x)
                right_x_dist = abs(points.x[r_idx] * dir_x + points.y[r_idx] * dir_y)
                if 0 < right_y_dist < 30 and 0 < right_x_dist < 20:
                    right_condition_met = True
                    continue_right = False  # 条件を満たしたら探索終了
        else:
            continue_right = False
    
    return left_condition_met and right_condition_met
    
    
def detectUshapedTile_ROI(angle:int, scan_map:ScanMap  | None = None) -> bool:
    """
    @brief コの字型（3辺が壁で囲まれている形状）のマスを検出する
    @param scan_map: 前処理済みのLiDARデータマップ
    @param angle: 探す方向の角度 (0: 前, 90: 左, 180: 後ろ, 270: 右 など)
    @return コの字型のタイルが検出された場合は True, それ以外は False
    """
    if scan_map is None:
        global pre_scan_map
        if pre_scan_map is None:
            raise ValueError("No preprocessed LiDAR data available. Please provide scan_map.")
        scan_map = pre_scan_map
    
    # 1. 探す方向の基準ベクトル
    dir_rad = math.radians(angle)
    dir_x = math.cos(dir_rad)
    dir_y = math.sin(dir_rad)
    
    # 2. 正面方向の距離がそもそも遠すぎる/近すぎる場合は早期リターン
    # コの字の中央付近を向いている角度の距離を見る
    center_dist = scan_map.ranges[int(angle) % 360]
    if not (30 < center_dist < 60):
        return False
        
    # 3. 探索する範囲のインデックスだけを切り出す (正面方向の ±45度)
    target_deg = int(angle) % 360
    idx_range = (target_deg + np.arange(-45, 46)) % 360
    valid_mask = scan_map.valid[idx_range]
    valid_indices = idx_range[valid_mask]
    
    if len(valid_indices) == 0:
        return False

    # 4. 対象となる点のグローバルX, Y座標を取得
    x_vals = scan_map.x[valid_indices]
    y_vals = scan_map.y[valid_indices]
    
    # 5. 指定した angle 方向を X軸、その直角方向を Y軸 とする「ローカル座標」に回転・変換
    local_x = x_vals * dir_x + y_vals * dir_y
    local_y = -x_vals * dir_y + y_vals * dir_x
    
    # --- 領域判定 ---
    # ロボットから見た隣のマスの想定エリア (1マス30cm)
    # 奥の壁：X方向 35cm〜55cm, Y方向 -12cm〜12cm
    front_wall_points = np.sum((35 < local_x) & (local_x < 55) & (-12 < local_y) & (local_y < 12))
    
    # 左の壁：X方向 15cm〜40cm, Y方向 10cm〜25cm
    left_wall_points = np.sum((15 < local_x) & (local_x < 45) & (5 < local_y) & (local_y < 25))
    
    # 右の壁：X方向 15cm〜40cm, Y方向 -25cm〜-10cm
    right_wall_points = np.sum((15 < local_x) & (local_x < 45) & (-25 < local_y) & (local_y < -5))
    
    REQUIRED_POINTS = 8
    
    return bool(front_wall_points >= REQUIRED_POINTS and 
            left_wall_points >= REQUIRED_POINTS and 
            right_wall_points >= REQUIRED_POINTS)

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