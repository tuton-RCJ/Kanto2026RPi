from . import mazeEnums
from .device import deviceEnums

TILE_SIZE_CM: int = 30  # タイル1枚のサイズ

STRAIGHT_GYRO_P_GAIN: float = 4
STRAIGHT_TOF_P_GAIN: float = 4

# USE_TURN_METHOD: mazeEnums.turnMethod = (
#     mazeEnums.turnMethod.ONLY_GYRO
# )  # 回転時の制御方法
# USE_MOVE_METHOD: mazeEnums.moveMethod = (
#     mazeEnums.moveMethod.SEE_FRONT
# )  # 直進時の制御方法
TURN_THRESHOLD_DEG_FIX: float = 2  # 回転時の誤差許容角度(調整時)
TURN_THRESHOLD_DEG: float = 6  # 回転時の誤差許容角度
GO_STRAIGHT_MAX_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 100,
    deviceEnums.Side.RIGHT: 100,
}  # 直進時のスピード

GO_BACKWARD_MAX_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: -100,
    deviceEnums.Side.RIGHT: -100,
}  # 後退時のスピード

GO_STRAIGHT_LOW_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 30,
    deviceEnums.Side.RIGHT: 30,
}  # ゆっくり直進時のスピード

GO_BACKWARD_LOW_SPEED: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: -30,
    deviceEnums.Side.RIGHT: -30,
}  # ゆっくり後退時のスピード


MOVE_THRESHOLD_CM: int = (
    28  # 直進時に　(前方との距離) mod 30 がこの値以上減少したら停止する
)
MOVE_STRAIGHT_THRESHOLD_CM: int = (
    15  # 直進時に前方との距離がこの値以下になったら停止する
)
WALL_DETECTION_THRESHOLD_CM: int = 30  # LiDARで壁を検出する閾値
CAR_HEIGHT: int = 10  # 車の高さ
USE_P_GAIN_FOR_TOF_DIST: int = 10  # tof を両側の壁距離制御に使用する際の閾値
WALL_DETECTION_RAMP_THRESHOULD_DIFF_CM: int = 15  # 坂検出をする、LiDAR距離とTSD10距離の差の閾値

# 直進中の壁追従(壁が近い時のみ)の制御パラメータ
WALL_FOLLOW_ENABLE_DIST_CM: float = 20  # 片側でもこの距離以下なら壁距離制御を有効化
WALL_FOLLOW_TARGET_DIST_CM: float = 14  # 片側のみ近い場合の目標距離
WALL_FOLLOW_P_GAIN: float = 2.0  # 壁距離制御の比例ゲイン(steer量)
WALL_FOLLOW_MAX_STEER: float = 10.0  # 壁距離制御のsteer上限(gyro優先のため抑える)
WALL_FOLLOW_GYRO_ERR_MAX_DEG: float = 5.0  # この角度誤差以内なら壁距離制御も併用

RAMP_END_THRESHOLD_CM: int = 28
TIMEOUT_FOR_TURNING_SEC: float = 5.0  # 回転動作のタイムアウト時間



MIN_TILE_DETECTION_THRESHOLD: int = 10  # タイル検出の最小回数閾値

TOF_BLACK_TILE_ESCAPE_DISTANCE_CM: int = 1  # 黒タイル検出後の後退許容誤差

BLACKTILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (10, 10, 15),
    (0, 0, 0),
)  # 黒タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
BLUETILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (10, 10, 120),
    (0, 0, 18),
)  # 青タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る
REDTILE_RGB: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (120, 20, 20),
    (15, 0, 0),
)  # 赤タイルと判定するRGB値の閾値, 一番大きな tuple のなかには二つ tuple が入る

# RAMP_DEG_THRESHOLD: float = 23  # 傾斜検出の閾値(度)
RAMP_DEG_THRESHOLD: float = 12  # 傾斜検出の閾値(度)
# +1: 正の signed roll を上りとして扱う, -1: 逆に扱う
RAMP_ROLL_SIGN_FOR_UP: int = 1
MIN_THRESHOLD_FOR_DIFF: float = 15

TURN_90_SEC: float = 0.35  # 90度回転にかかる時間
MOVE_STRAIGHT_SEC: float = 0.80  # 1マス直進にかかる時間
MIN_MOVE_STRAIGHT_SEC: float = 0.6  # 最短で1マス直進にかかると考えられる時間
MOVETILE_TIMEOUT_SEC: float = 10  # 1マス移動のタイムアウト時間
BLUE_TILE_WAIT_SEC: float = 5 # 青タイルの待機時間

USE_PD_FOR_TURNING: bool = True  # 回転時にPD制御を使用するかどうか

TURN_SPD: int = 80  # 回転時のモーター速度

TURN_SPD_SLOW: int = 10  # 回転時のモーター速度(角度補正時、低速)

TURN_P = 2  # 回転制御の比例ゲイン
TURN_I = 0.00  # 回転制御の積分ゲイン
TURN_D = 0.00  # 回転制御の微分ゲイン



DEBUG_MODE: bool = True  # デバッグモードの有効化


# 45度回転の時の被災者検出のパラメータ
USE_45_TURN_WITH_VICTIM_CHECK: bool = False  # 45度回転の時に被災者検出をするかどうか
DETECT_VICTIM_45_CHECK_TIME_SEC: float = 0.5  # 45度回転した後、被災者検出を行う秒数

USE_SLOW_DOWN_FOR_VICTIM_DETECTION_WHEN_TURNING: bool = True  # 回転中に被災者検出をする際に、回転速度を落とすかどうか, USE_45_TURN_WITH_VICTIM_CHECK が False の場合のみ有効
SLOW_DOWN_FOR_VICTIM_DETECTION_WHEN_TURNING_SPEED: int = 30  # 回転中に被災者検出をする際の回転速度, USE_SLOW_DOWN_FOR_VICTIM_DETECTION_WHEN_TURNING が True の場合のみ有効

STAIR_THRESHOLD_CM: float = 5.0  # 階段判定のしきい値。鉛直距離の合計がこの値を下回っていたら、坂判定をしていても階段とみなす



##### 坂道例外処理


TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_RAMP: bool = True # 坂道の途中で前に壁を検出した時に引き返すか
TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_UP_RAMP_THRESHOLD_CM: float = 20.0 # 上り坂道の途中で前に壁を検出した時に引き返すかのしきい値
TURN_BACK_WHEN_FRONT_WALL_DETECTED_ON_DOWN_RAMP_THRESHOLD_CM: float = 20.0 # 下り坂道の途中で前に壁を検出した時に引き返すかのしきい値


###### Dangerous Zone関連

AVOID_DANGEROUS_ZONE: bool = True  # Dangerous Zoneを避けるかどうか
AVOID_SLOPE_IN_DANGEROUS_ZONE: bool = False  # Dangerous Zone内の坂を避けるかどうか


###### レスキューキット投下

DROP_ONLY_ONE_KIT_FOR_HARMED_COGNITIVE : bool = False  # 2点のCognitive Targetに1つのキットしか投下しない

BACKWARD_AFTER_DROP_KIT : bool = True  # 救助キット投下後に少し後退するかどうか
BACKWARD_AFTER_DROP_KIT_TIME_SEC : float = 0.5  # 救助キット投下後に後退する時間 (スピードは GO_BACKWARD_LOW_SPEED を使用)

USE_SAME_COLOR_FOR_VICTIM_DETECTION_LED_BLINK : bool = False  # 被災者検出用LEDを点滅させる際に、同じ色で点滅させるかどうか

DEFAULT_RESCUE_KIT_COUNT: dict[deviceEnums.Side, int] = {
    deviceEnums.Side.LEFT: 4,
    deviceEnums.Side.RIGHT: 4,
}  # 各サイドの初期レスキューキットの数

TURN_ANGLE_WHEN_DROP_MULTIPLE_KITS: int = (
    0  # 複数の救助キットを投下する際に回転する角度
)

###### エラーハンドリング
DESTROY_ALL_INTERNAL_MAP_WHEN_WALL_DETECTION_ERROR: bool = True  # 壁検出エラーが発生した場合に、内部マップを全て破棄するかどうか



###### Dijkstraのコスト計算の定数値。実測値を代入する。
DIJKSTRA_COST_STRAIGHT_ONE_TILE: float = 1.0  # 直進のコスト
DIJKSTRA_COST_TURN_90_DEG: float = 1.0  # 回転のコスト
DIJKSTRA_COST_BLUE_TILE: float = 5.0  # 青タイルの追加コスト



####### 帰還開始判定

RETURN_JUDGE_MODE: mazeEnums.returnJudgeMode = mazeEnums.returnJudgeMode.ONLY_TIME_BASED  # 帰還開始判定のモード

GAME_TIME_SEC: float = 480.0  # 制限時間 (秒)

RETURN_TIME_THRESHOLD_SEC: float = 360.0  # この時間たったら帰還開始と判断する閾値 (RETURN_JUDGE_MODE が ONLY_TIME_BASED の場合に使用)

RETURN_TIME_WITH_DISTANCE_THRESHOLD_SEC: float = 400.0  # 現在の時刻 + 帰還に必要な時間がこの値を超えると帰還開始と判断する閾値 (RETURN_JUDGE_MODE が TIME_BASED_WITH_DISTANCE の場合に使用)
