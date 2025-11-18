from enum import Enum

class absDirection(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


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
    UNKNOWN = 0
    EMPTY = 1
    RED = 2
    BLACK = 3
    BLUE = 4

