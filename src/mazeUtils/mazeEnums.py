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
    
    def opposite(self):
        if self == absDirection.NORTH:
            return absDirection.SOUTH
        elif self == absDirection.EAST:
            return absDirection.WEST
        elif self == absDirection.SOUTH:
            return absDirection.NORTH
        elif self == absDirection.WEST:
            return absDirection.EAST
        else:
            raise ValueError("Invalid absDirection: {}".format(self))
        
class wallType(Enum):
    UNKNOWN = 0
    NO_WALL = 1
    WALL = 2
    U_VICTIM = 3
    S_VICTIM = 4
    H_VICTIM = 5
    G_VICTIM = 6
    Y_VICTIM = 7
    R_VICTIM = 8
    OBSTACLE_WALL = 9

class tileType(Enum):
    UNKNOWN = "U"
    EMPTY = "E"
    RED = "R"
    BLACK = "B"
    BLUE = "L"
    START = "S"
    SILVER = "V"
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

class returnJudgeMode(Enum):
    ONLY_TIME_BASED = 0 # 制限時間のみで帰還開始を判断する
    ALL_TILES_EXPLORED = 1 # 全てのタイルが探索されたら帰還開始
    TIME_BASED_WITH_DISTANCE = 2 # スタートタイルに帰還するのにかかる時間が制限時間を超えると判断したら帰還開始

directionToDelta = {
    absDirection.NORTH: (0, -1),
    absDirection.EAST: (1, 0),
    absDirection.SOUTH: (0, 1),
    absDirection.WEST: (-1, 0)
}