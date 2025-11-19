import mazeEnums
from device import deviceEnums

P_GAIN: float = 0.2
I_GAIN: float = 0.0
D_GAIN: float = 0.01

USE_TURN_METHOD: mazeEnums.turnMethod = mazeEnums.turnMethod.ONLY_GYRO # 回転時の制御方法
USE_MOVE_METHOD: mazeEnums.moveMethod = mazeEnums.moveMethod.SEE_FRONT # 直進時の制御方法
TURN_THRESHOLD_DEG: int = 3 # 回転時の誤差許容角度

GO_STRAIGHT_MAX_SPEED: dict[deviceEnums.Side, int] = {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50} # 直進時のスピード

NEWS_DIRECTION = [mazeEnums.absDirection.NORTH.value, mazeEnums.absDirection.EAST.value, mazeEnums.absDirection.SOUTH.value, mazeEnums.absDirection.WEST.value]

MOVE_THRESHOLD_CM: int = 10  # 直進時に　(前方との距離) mod 30 がこの値以上減少したら停止する