from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils.device import photoReflector
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    fr = photoReflector.PhotoReflector()
    while True:
        if fr.isReflecting():
            print("Reflecting")
        else:
            print("Not Reflecting")
        time.sleep(0.5)
if __name__ == "__main__":
    main()