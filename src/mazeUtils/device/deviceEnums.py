from enum import Enum


class Side(Enum):
    LEFT = 0
    RIGHT = 1

    def opposite(self):
        if self == Side.LEFT:
            return Side.RIGHT
        else:
            return Side.LEFT


class UnitVStatus(Enum):
    NOTHING = 0
    U_VICTIM = 1
    S_VICTIM = 2
    H_VICTIM = 3
    G_VICTIM = 4
    Y_VICTIM = 5
    R_VICTIM = 6


class ActuatorControlType(Enum):
    STS_MOTOR = bytes([1])
    RESCUE_KIT = bytes([2])
    LED = bytes([3])
    BUZZER = bytes([4])
    CAMLED = bytes([5])
    UNITV_45_MODE = bytes([6])
    def dataLength(self):
        if self == ActuatorControlType.STS_MOTOR:
            return 2
        elif self == ActuatorControlType.RESCUE_KIT:
            return 2
        elif self == ActuatorControlType.LED or self == ActuatorControlType.CAMLED:
            return 3
        elif self == ActuatorControlType.BUZZER:
            # 可変長: 音符数(1byte) + 各音符(周波数2byte, 長さ2byte) * N
            return -1
        elif self == ActuatorControlType.UNITV_45_MODE:
            return 1
        return -1
