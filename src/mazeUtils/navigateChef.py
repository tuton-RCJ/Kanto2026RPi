import ydlidar
from . import mazeConstraints, mazeEnums, mazeMap, moveTile
from .device import LiDAR, deviceConstraints, deviceEnums, stm, camera, buzzerSongs
from config import get_logger
import time
import threading
import numpy as np
import math
from collections import defaultdict

def goToFirstBlackTile(stmInstance, mapInstance, lidarInstance):
    """
    @brief 最初の黒タイルまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go to the first black tile.")
    assert mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 0, "Current position must be (0, 0) at the start."
    for i in range(2):
        moveTile.movetile(mazeEnums.absDirection.NORTH, stmInstance, mapInstance, lidarInstance)
    