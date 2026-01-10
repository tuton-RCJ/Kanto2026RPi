from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils.device import camera
from mazeUtils.device import photoReflector
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    t = time.time()
    stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50})
    while time.time() - t < 1:
        pass
    stmInstance.sts3032.stop()
if __name__ == "__main__":
    main()