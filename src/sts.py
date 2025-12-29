from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    # LiDARInstance = LiDAR.initializeLidar()
    stmInstance.buzzer.playMusic(buzzerSongs.matuken)
    colorsensorInstance = colorsensor.ColorSensor()
    
    while True:
        colorsensorInstance.update()
        stmInstance.update()
        print(moveTile.detectTileColor())
if __name__ == "__main__":
    main()