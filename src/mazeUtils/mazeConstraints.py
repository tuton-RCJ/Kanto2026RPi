from . import mazeEnums
from .device import deviceEnums

TILE_SIZE_CM: int = 30  # タイル1枚のサイズ

STRAIGHT_GYRO_P_GAIN: float = 0.5
STRAIGHT_TOF_P_GAIN: float = 0.3

USE_TURN_METHOD: mazeEnums.turnMethod = (
    mazeEnums.turnMethod.ONLY_GYRO
)  # 回転時の制御方法
USE_MOVE_METHOD: mazeEnums.moveMethod = (
    mazeEnums.moveMethod.SEE_FRONT
)  # 直進時の制御方法
TURN_THRESHOLD_DEG_FIX: float = 0.2  # 回転時の誤差許容角度(調整時)
TURN_THRESHOLD_DEG: float = 10  # 回転時の誤差許容角度
GO_STRAIGHT_MAX_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 50,
    deviceEnums.Side.RIGHT: 50,
}  # 直進時のスピード

GO_STRAIGHT_LOW_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 30,
    deviceEnums.Side.RIGHT: 30,
}  # ゆっくり直進時のスピード

NEWS_DIRECTION = [
    mazeEnums.absDirection.NORTH.value,
    mazeEnums.absDirection.EAST.value,
    mazeEnums.absDirection.SOUTH.value,
    mazeEnums.absDirection.WEST.value,
]

MOVE_THRESHOLD_CM: int = (
    28  # 直進時に　(前方との距離) mod 30 がこの値以上減少したら停止する
)
MOVE_STRAIGHT_THRESHOLD_CM: int = (
    15  # 直進時に前方との距離がこの値以下になったら停止する
)
WALL_DETECTION_THRESHOLD_CM: int = 22  # LiDARで壁を検出する閾値
CAR_HEIGHT: int = 10  # 車の高さ
USE_P_GAIN_FOR_TOF_DIST: int = 10  # tof を両側の壁距離制御に使用する際の閾値

# 直進中の壁追従(壁が近い時のみ)の制御パラメータ
WALL_FOLLOW_ENABLE_DIST_CM: float = 20  # 片側でもこの距離以下なら壁距離制御を有効化
WALL_FOLLOW_TARGET_DIST_CM: float = 16.5  # 片側のみ近い場合の目標距離
WALL_FOLLOW_P_GAIN: float = 2.0  # 壁距離制御の比例ゲイン(steer量)
WALL_FOLLOW_MAX_STEER: float = 10.0  # 壁距離制御のsteer上限(gyro優先のため抑える)
WALL_FOLLOW_GYRO_ERR_MAX_DEG: float = 5.0  # この角度誤差以内なら壁距離制御も併用

RAMP_END_THRESHOLD_CM: int = 28
TIMEOUT_FOR_TURNING_SEC: float = 5.0  # 回転動作のタイムアウト時間

TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS: int = (
    15  # 複数の救助キットを投下する際に回転する角度
)

MIN_TILE_DETECTION_THRESHOLD: int = 10  # タイル検出の最小回数閾値

TOF_BLACK_TILE_ESCAPE_DISTANCE_CM: int = 1  # 黒タイル検出後の後退許容誤差

BLACKTILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (10, 10, 15),
    (0, 0, 0),
)  # 黒タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
BLUETILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (10, 30, 120),
    (0, 0, 30),
)  # 青タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
REDTILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (120, 20, 20),
    (30, 0, 0),
)  # 赤タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る

RAMP_DEG_THRESHOLD: float = 23  # 傾斜検出の閾値(度)
# +1: 正の signed roll を上りとして扱う, -1: 逆に扱う
RAMP_ROLL_SIGN_FOR_UP: int = 1
MIN_THRESHOLD_FOR_DIFF: float = 15

TURN_90_SEC: float = 0.35  # 90度回転にかかる時間
MOVE_STRAIGHT_SEC: float = 1.46  # 1マス直進にかかる時間
MIN_MOVE_STRAIGHT_SEC: float = 0.8  # 最短で1マス直進にかかると考えられる時間
MOVETILE_TIMEOUT_SEC: float = 10  # 1マス移動のタイムアウト時間

USE_PD_FOR_TURNING: bool = True  # 回転時にPD制御を使用するかどうか

USE_SPEED_CONTROL_FOR_STRAIGHT: bool = True  # 直進時に速度制御するかどうか


TURN_SPD: int = 80  # 回転時のモーター速度

TURN_P = 2  # 回転制御の比例ゲイン
TURN_I = 0.00  # 回転制御の積分ゲイン
TURN_D = 0.00  # 回転制御の微分ゲイン

RAMP_TOF_THRESHOLD = 4
JUDGE_RAMP_LIDAR_THRESHOLD = 50

DEFAULT_RESCUE_KIT_COUNT: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 6,
    deviceEnums.Side.RIGHT: 6,
}  # 各サイドの初期レスキューキットの数

DEBUG_MODE: bool = True  # デバッグモードの有効化
