from enum import Enum


class Side(Enum):
    LEFT = 0
    RIGHT = 1


class UnitVStatus(Enum):
    UNKNOWN = 0
    NOTHING = 1


class ActuatorControlType(Enum):
    STS_MOTOR = bytes([1])
    RESCUE_KIT = bytes([2])
    LED = bytes([3])
    BUZZER = bytes([4])

    def dataLength(self):
        if self == ActuatorControlType.STS_MOTOR:
            return 2
        elif self == ActuatorControlType.RESCUE_KIT:
            return 2
        elif self == ActuatorControlType.LED:
            return 3
        elif self == ActuatorControlType.BUZZER:
            return 1
