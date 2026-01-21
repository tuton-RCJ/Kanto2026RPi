from . import mazeEnums
from .device import deviceEnums

STRAGIHT_GYRO_P_GAIN: float = 0.5
STRAGIHT_TOF_P_GAIN: float = 1

START_LEVEL: int = 0  # 立体交差用の初期レベル
RAMP_LEVEL_BASE: int = 10000  # 坂専用レベルの開始ID
TILE_LENGTH_CM: float = 30.0  # タイル一辺の長さ(cm)
LEVEL_HEIGHT_CM: float = 20.0  # レベル差の高さ(cm)
RAMP_END_LEVEL_SNAP_RATIO: float = 0.5  # 坂道終了時に残差を丸める比率

USE_TURN_METHOD: mazeEnums.turnMethod = mazeEnums.turnMethod.ONLY_GYRO # 回転時の制御方法
USE_MOVE_METHOD: mazeEnums.moveMethod = mazeEnums.moveMethod.SEE_FRONT # 直進時の制御方法
MEASURING_DIST_METHOD: mazeEnums.measuringDistMethod = mazeEnums.measuringDistMethod.LIDAR # 壁距離の測定方法
TURN_THRESHOLD_DEG_FIX: float = 0.2 # 回転時の誤差許容角度(調整時)
TURN_THRESHOLD_DEG: float = 10  # 回転時の誤差許容角度
VICTIM_TURN_DOUBLE_ADD_DIFF_DEG: float = 20.0  # 回転中、カメラ向きが方位の中間に近い場合に近い2方向へ被災者情報を登録する許容差(度)
GO_STRAIGHT_MAX_SPEED: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50} # 直進時のスピード

GO_STRAIGHT_LOW_SPEED: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 30, deviceEnums.Side.RIGHT: 30} # ゆっくり直進時のスピード

NEWS_DIRECTION = [mazeEnums.absDirection.NORTH.value, mazeEnums.absDirection.EAST.value, mazeEnums.absDirection.SOUTH.value, mazeEnums.absDirection.WEST.value] 

MOVE_THRESHOLD_CM: int = 28  # 直進時に　(前方との距離) mod 30 がこの値以上減少したら停止する
MOVE_STRAIGHT_THRESHOLD_CM: int = 16 # 直進時に前方との距離がこの値以下になったら停止する
WALL_DETECTION_THRESHOLD_CM: int = 25  # LiDARで壁を検出する閾値
USE_P_GAIN_FOR_TOF_DIST: int = 10  # tof を両側の壁距離制御に使用する際の閾値

# 直進中の壁追従(壁が近い時のみ)の制御パラメータ
WALL_FOLLOW_ENABLE_DIST_CM: int = 20  # 片側でもこの距離以下なら壁距離制御を有効化
WALL_FOLLOW_TARGET_DIST_CM: int = 16.5  # 片側のみ近い場合の目標距離
WALL_FOLLOW_P_GAIN: float = 2.0  # 壁距離制御の比例ゲイン(steer量)
WALL_FOLLOW_MAX_STEER: float = 10.0  # 壁距離制御のsteer上限(gyro優先のため抑える)
WALL_FOLLOW_GYRO_ERR_MAX_DEG: float = 5.0  # この角度誤差以内なら壁距離制御も併用

TIMEOUT_FOR_TURNING_SEC: float = 5.0  # 回転動作のタイムアウト時間

TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS: int = 15  # 複数の救助キットを投下する際に回転する角度

TOF_BLACK_TILE_ESCAPE_DISTANCE_CM: int = 1  # 黒タイル検出後の後退許容誤差

BLACKTILE_RGB: tuple[tuple[int, int, int]] = ((15, 15, 15), (0, 0, 0))  # 黒タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
BLUETILE_RGB: tuple[tuple[int, int, int]] = ((10, 30, 120), (0, 0, 40))  # 青タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
REDTILE_RGB: tuple[tuple[int, int, int]] = ((120, 20, 20), (30, 0, 0))  # 赤タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る

RAMP_DEG_THRESHOLD: float = 15.0  # 傾斜検出の閾値(度)
RAMP_ROLL_SIGN_THRESHOLD_DEG: float = 5.0  # 上り/下り判定でロールの符号を信頼する最小角度
MIN_THERESHOULD_FOR_DIFF: float = 15 # 階段時、壁が現れたと判断するための1ループにおける距離差の閾値(cm)
THRESHOLD_SEE_CAM: float = 0.2 # カメラをタイルのどこから見るか (0 ~ 1, 割合)

TURN_90_SEC: float = 0.35  # 90度回転にかかる時間
MOVE_STRAIGHT_SEC: float = 1.3  # 1マス直進にかかる時間
MOVETILE_TIMEOUT_SEC: float = 10  # 1マス移動のタイムアウト時間

USE_SPEED_CONTROL_FOR_STRAIGHT: bool = True  # 直進時に速度制御するかどうか

USE_FORWARD_LIDAR: bool = False  # 直進時にLiDARを使用するかどうか

TURN_P = 2  # 回転制御の比例ゲイン
TURN_I = 0.00  # 回転制御の積分ゲイン
TURN_D = 0.00  # 回転制御の微分ゲイン

DEFAULT_RESCUE_KIT_COUNT: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 6, deviceEnums.Side.RIGHT: 6}  # 各サイドの初期レスキューキットの数

DEBUG_MODE: bool = False  # デバッグモードの有効化
ENABLE_CLI_MAP_VIEW: bool = True  # CLIで表示階を切替える