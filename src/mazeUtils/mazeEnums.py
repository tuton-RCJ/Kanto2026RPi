from enum import Enum
from .device import deviceEnums

class absDirection(Enum):
    NORTH = 0
    EAST = 270
    SOUTH = 180
    WEST = 90

    # 足し算と引き算
    def __add__(self, other):
        if isinstance(other, int):
            new_value = (self.value + other) % 360
            for direction in absDirection:
                if direction.value == new_value:
                    return direction
            raise ValueError("No matching absDirection for value {}".format(new_value))
        if isinstance(other, absDirection):
            new_value = (self.value + other.value) % 360
            for direction in absDirection:
                if direction.value == new_value:
                    return direction
            raise ValueError("No matching absDirection for value {}".format(new_value))
        return NotImplemented
    
    def __sub__(self, other):
        if isinstance(other, int):
            new_value = (self.value - other) % 360
            for direction in absDirection:
                if direction.value == new_value:
                    return direction
            raise ValueError("No matching absDirection for value {}".format(new_value))
        if isinstance(other, absDirection):
            new_value = (self.value - other.value) % 360
            for direction in absDirection:
                if direction.value == new_value:
                    return direction
            raise ValueError("No matching absDirection for value {}".format(new_value))
        return NotImplemented
        
class wallType(Enum):
    UNKNOWN = 0
    NO_WALL = 1
    WALL = 2
    H_VICTIM = 3
    S_VICTIM = 4
    U_VICTIM = 5
    Y_VICTIM = 6
    G_VICTIM = 7
    R_VICTIM = 8

class tileType(Enum):
    UNKNOWN = "U"
    EMPTY = "E"
    RED = "R"
    BLACK = "B"
    BLUE = "L"
    START = "S"
    def __str__(self):
        return self.value

class turnMethod(Enum):
    ONLY_LiDAR = 0
    ONLY_GYRO = 1
    LiDAR_AND_GYRO = 2

class turnDirection(Enum):
    LEFT = -1
    RIGHT = 1

class moveMethod(Enum):
    SEE_CORNER = 0
    SEE_FRONT = 1

BLACK_TILE_BRIGHTNESS_THRESHOLD: int = 50

