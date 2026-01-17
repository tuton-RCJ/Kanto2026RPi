"""
@brief ハードウェアデバイスの初期化とインスタンス管理を行うモジュール
"""

from __future__ import annotations

import threading

import ydlidar

from . import mazeConstraints, mazeMap
from .device import LiDAR, colorsensor, photoReflector, stm


class Robot:
    """
    @brief ハードウェアデバイスの初期化とインスタンス保持を行うクラス
    """

    def __init__(self) -> None:
        """
        @brief Robotクラスの初期化。全デバイスを初期化する
        """
        self.stmInstance: stm.STM = stm.STM()
        self.lidarInstance: ydlidar.CYdLidar = LiDAR.initializeLidar()
        self.colorSensorInstance: colorsensor.ColorSensor = colorsensor.ColorSensor()
        self.photoReflectorInstance: photoReflector.PhotoReflector = photoReflector.PhotoReflector()
        self.mapInstance: mazeMap.mazeMap = mazeMap.mazeMap()

        self._lidarWorker: LiDAR.AsyncLidarScanWorker | None = None
        self._lidarWorkerLock = threading.Lock()

    def getLidarWorker(self) -> LiDAR.AsyncLidarScanWorker:
        """
        @brief LiDARスキャンワーカーを取得する。未生成なら生成する。
        @return AsyncLidarScanWorkerインスタンス
        """
        with self._lidarWorkerLock:
            if self._lidarWorker is None:
                self._lidarWorker = LiDAR.AsyncLidarScanWorker(self.lidarInstance, debugMode=mazeConstraints.DEBUG_MODE)
            return self._lidarWorker

    def update(self) -> bool:
        """
        @brief STMからセンサ値を更新する
        @return 更新成功ならTrue
        """
        return self.stmInstance.update()

    def isToggleSwitchOn(self) -> bool:
        """
        @brief トグルスイッチの状態を取得する
        @return トグルスイッチがオンならTrue
        """
        return self.stmInstance.switch.getToggleSwitch1()

    def isPushSwitchOn(self) -> bool:
        """
        @brief プッシュスイッチの状態を取得する
        @return プッシュスイッチがオンならTrue
        """
        return self.stmInstance.switch.getPushSwitch1()

    def setLedColor(self, color: list[int]) -> None:
        """
        @brief LEDの色を設定する
        @param color RGB値のリスト [R, G, B]
        """
        self.stmInstance.led.setColor(*color)

    def turnOffLed(self) -> None:
        """
        @brief LEDを消灯する
        """
        self.setLedColor([0, 0, 0])

    def stopMotors(self) -> bool:
        """
        @brief モータを停止する
        @return 成功ならTrue
        """
        return self.stmInstance.sts3032.stop()

    def shutdown(self) -> None:
        """
        @brief 全デバイスをシャットダウンする
        """
        with self._lidarWorkerLock:
            if self._lidarWorker is not None:
                self._lidarWorker.stop()
                self._lidarWorker = None

        self.stopMotors()
        self.turnOffLed()

        LiDAR.liDARShutdown(self.lidarInstance)
