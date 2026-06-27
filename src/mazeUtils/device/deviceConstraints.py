
LOADCELL_THRESHOLD = 100  # ADCのしきい値


### LiDARの壁・障害物判定のしきい値
WALL_DETECTION_THRESHOLD_CM: int = 30  # LiDARで壁を検出する閾値
WALL_DETECTION_THRESHOLD_CM_SIDE: int = 25 # 側方の壁の検出閾値
SIDE_OBSTACLE_DETECTION_THRESHOLD_CM: int = 30  # LiDARで側方障害物を検出する閾値
SIDE_OBSTACLE_DETECTION_X_LIMIT_CM: int = 10 # LiDARで側方障害物を検出する際のX軸方向の制限(cm) LiDARの原点から見て、±X_LIMITの範囲内にある障害物のみを検出する
CENTER_OBSTACLE_DETECTION_ANGLE_LIMIT_DEG: int = 5 # LiDARで正面障害物を検出する際の角度制限(°) LiDARの原点から見て、±ANGLE_LIMITの範囲で認識したら障害物と判断。
CENTER_OBSTACLE_DETECTION_X_LIMIT_CM: int = 10 # LiDARで正面障害物を検出する際のX軸方向の制限(cm) LiDARの原点から見て、±X_LIMITの範囲内にある障害物のみを検出する

CONNECTED_COMPONENT_THRESHOLD_CM: int = 10  # 連結成分の閾値(cm) この値以下の距離差の場合連結させる
WALL_DETECTION_CONNECTED_COMPONENT_THRESHOLD_CM: int = 20  # LiDARで壁を検出する際の連結成分のしきい値(cm) 連結成分の長さ(X軸方向)がこの値以上のとき壁と判断