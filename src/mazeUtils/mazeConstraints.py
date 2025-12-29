from . import mazeEnums
from .device import deviceEnums

STRAGIHT_GYRO_P_GAIN: float = 2
STRAGIHT_TOF_P_GAIN: float = 2

USE_TURN_METHOD: mazeEnums.turnMethod = mazeEnums.turnMethod.ONLY_GYRO # 回転時の制御方法
USE_MOVE_METHOD: mazeEnums.moveMethod = mazeEnums.moveMethod.SEE_FRONT # 直進時の制御方法
TURN_THRESHOLD_DEG_FIX: float = 0.2 # 回転時の誤差許容角度(調整時)
TURN_THRESHOLD_DEG: float = 10  # 回転時の誤差許容角度
GO_STRAIGHT_MAX_SPEED: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50} # 直進時のスピード

GO_STRAIGHT_LOW_SPEED: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 30, deviceEnums.Side.RIGHT: 30} # ゆっくり直進時のスピード

NEWS_DIRECTION = [mazeEnums.absDirection.NORTH.value, mazeEnums.absDirection.EAST.value, mazeEnums.absDirection.SOUTH.value, mazeEnums.absDirection.WEST.value] 

MOVE_THRESHOLD_CM: int = 28  # 直進時に　(前方との距離) mod 30 がこの値以上減少したら停止する
MOVE_STRAIGHT_THRESHOLD_CM: int = 14  # 直進時に前方との距離がこの値以下になったら停止する
WALL_DETECTION_THRESHOLD_CM: int = 30  # LiDARで壁を検出する閾値
USE_P_GAIN_FOR_TOF_DIST: int = 10  # tof を両側の壁距離制御に使用する際の閾値

TOF_BLACK_TILE_ESCAPE_DISTANCE_CM: int = 1  # 黒タイル検出後の後退許容誤差

BLACKTILE_RGB: tuple[tuple[int, int, int]] = ((10, 12, 20), (0, 0, 0))  # 黒タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
BLUETILE_RGB: tuple[tuple[int, int, int]] = ((20, 40, 150), (3, 3, 60))  # 青タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
SILVERTILE_RGB: tuple[tuple[int, int, int]] = ((255, 255, 255), (80, 100, 70))  # 銀タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る

RAMP_DEG_THRESHOLD: float = 15.0  # 傾斜検出の閾値(度)

TURN_90_SEC: float = 0.35  # 90度回転にかかる時間
MOVE_STRAIGHT_SEC: float = 1.1  # 1マス直進にかかる時間
MOVETILE_TIMEOUT_SEC: float = 10  # 1マス移動のタイムアウト時間

USE_PD_FOR_TURNING: bool = True  # 回転時にPD制御を使用するかどうか

USE_SPEED_CONTROL_FOR_STRAIGHT: bool = True  # 直進時に速度制御するかどうか

USE_FORWARD_LIDAR: bool = False  # 直進時にLiDARを使用するかどうか

TURN_P = 2  # 回転制御の比例ゲイン
TURN_I = 0.00  # 回転制御の積分ゲイン
TURN_D = 0.00  # 回転制御の微分ゲイン

DEFAULT_RESCUE_KIT_COUNT: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 6, deviceEnums.Side.RIGHT: 6}  # 各サイドの初期レスキューキットの数

DEBUG_MODE: bool = False  # デバッグモードの有効化