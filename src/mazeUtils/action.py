"""
@brief アクチュエータへの命令を行うモジュール
"""

from __future__ import annotations

import time

from . import mazeConstraints
from .device import buzzerSongs, deviceEnums
from .robot import Robot


class Action:
    """
    @brief アクチュエーターへの命令を管理するクラス
    """

    def __init__(self, robotInstance: "Robot") -> None:
        """
        @brief Actionクラスの初期化
        @param robotInstance Robotインスタンス
        """
        self._robot = robotInstance

    def setMotor(self, speeds: dict[deviceEnums.Side, int]) -> bool:
        """
        @brief モータ速度を設定する
        @param speeds 各サイドのモータ速度の辞書
        @return 成功ならTrue
        """
        return self._robot.stmInstance.sts3032.setMotorSpeed(speeds)

    def stop(self) -> bool:
        """
        @brief モータを停止する
        @return 成功ならTrue
        """
        return self._robot.stmInstance.sts3032.stop()

    def turnRight(self, speed: int) -> bool:
        """
        @brief 右旋回する
        @param speed モータ速度
        @return 成功ならTrue
        """
        return self._robot.stmInstance.sts3032.turnRight(speed)

    def turnLeft(self, speed: int) -> bool:
        """
        @brief 左旋回する
        @param speed モータ速度
        @return 成功ならTrue
        """
        return self._robot.stmInstance.sts3032.turnLeft(speed)

    def escapeObstacle(self, pressedSide: deviceEnums.Side) -> None:
        """
        @brief 障害物に対する回避動作を行う
        @param pressedSide 押されたサイド
        """
        print(f"Escape from obstacle on {pressedSide} side")
        self.setMotor({deviceEnums.Side.LEFT: -30, deviceEnums.Side.RIGHT: -30})
        time.sleep(0.2)

        if pressedSide == deviceEnums.Side.LEFT:
            self.turnRight(30)
        else:
            self.turnLeft(30)
        time.sleep(0.2)

        self.stop()

    def turnOnLED(self, color: list[int]) -> None:
        """
        @brief LEDを点灯する
        @param color RGB値のリスト [R, G, B]
        """
        self._robot.setLedColor(color)

    def turnOffLED(self) -> None:
        """
        @brief LEDを消灯する
        """
        self._robot.turnOffLed()

    def flashLED(self, loopCount: int, intervalSec: float, color: list[int]) -> None:
        """
        @brief LEDを点滅させる
        @param loopCount 点滅回数
        @param intervalSec 点灯と消灯の間隔（秒）
        @param color RGB値のリスト [R, G, B]
        """
        for _ in range(loopCount):
            if self._robot.isToggleSwitchOn():
                self.turnOffLED()
                return

            self.turnOnLED(color)
            t = time.time()
            while time.time() - t < intervalSec:
                self._robot.update()
                if self._robot.isToggleSwitchOn():
                    self.turnOffLED()
                    return

            self.turnOffLED()
            t = time.time()
            while time.time() - t < intervalSec:
                self._robot.update()
                if self._robot.isToggleSwitchOn():
                    self.turnOffLED()
                    return

    def dropKit(self, side: deviceEnums.Side, count: int) -> None:
        """
        @brief 救助キットを投下する（物理動作のみ）
        @param side 投下するサイド
        @param count 投下する個数
        """
        self._robot.stmInstance.rescuekitservo.dropRescueKit(count, side)
        
    def playMusic(self, music: buzzerSongs.MusicData) -> bool:
        """
        @brief ブザーで音楽を再生する
        @param music 再生する音楽データ
        @return 成功ならTrue
        """
        return self._robot.stmInstance.buzzer.playMusic(music)
