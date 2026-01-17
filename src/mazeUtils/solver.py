"""
@brief アプリケーションの最上位管理と探索アルゴリズム実行を行うモジュール
"""

from __future__ import annotations

import time
from . import mazeEnums
from .device import buzzerSongs
from .robot import Robot
from .perception import Perception
from .action import Action
from .navigator import Navigator



class MazeSolver:
    """
    @brief 迷路探索アルゴリズムを管理するクラス
    """

    def __init__(self) -> None:
        """
        @brief MazeSolverクラスの初期化。全コンポーネントを生成する
        """
        self._robot = Robot()
        self._perception = Perception(self._robot)
        self._action = Action(self._robot)
        self._navigator = Navigator(self._robot, self._perception, self._action)

    def run(self) -> None:
        """
        @brief 探索アルゴリズムのメインループを実行する
        """
        try:
            self._waitForStart()
            self._calibrateGyro()
            self._explorationLoop()

            while self._returnToStart():
                pass
            
            self._goalPerformance()
        except Exception as e:
            print(f"Error during maze solving: {e}")
            raise
        finally:
            self._robot.shutdown()

    def _waitForStart(self) -> None:
        """
        @brief 開始スイッチが押されるまで待機する
        """
        print("Waiting for start switch...")

        while True:
            self._robot.update()
            if self._perception.isStartPressed():
                break
            time.sleep(0.05)

        print("Start switch pressed. Beginning exploration.")

    def _calibrateGyro(self) -> None:
        """
        @brief ジャイロセンサのキャリブレーションを行う
        """
        print("Calibrating gyro...")

        self._robot.update()
        currentGyro = self._robot.stmInstance.gyro.data
        self._robot.stmInstance.gyro.setOffset(currentGyro)

        print(f"Gyro calibrated. Offset: heading={currentGyro.heading}, pitch={currentGyro.pitch}, roll={currentGyro.roll}")

    def _explorationLoop(self) -> None:
        """
        @brief 探索ループを実行する。未探索タイルがなくなるまで繰り返す
        """
        print("Starting exploration loop...")
        mapInstance = self._robot.mapInstance

        while True:
            if self._perception.isSwitchPressed():
                print("Toggle switch detected. Handling Lack of Progress.")
                self._handleLackOfProgress()
                continue

            nextDirections = mapInstance.getNearestUnexploredTile()

            if nextDirections is None:
                print("No more unexplored tiles. Exploration complete.")
                break

            print(f"Next path: {[d.name for d in nextDirections]}")

            for direction in nextDirections:
                if self._perception.isSwitchPressed():
                    break

                isBlack, isLoP = self._navigator.moveTile(direction)

                if isLoP:
                    print("Lack of Progress detected during movement.")
                    self._handleLackOfProgress()
                    break

                if isBlack:
                    print("Black tile detected. Recalculating path.")
                    break

            print(mapInstance.renderKnownTileAndWall())
            
        self._action.playMusic(buzzerSongs.hotaru)

    def _handleLackOfProgress(self) -> None:
        """
        @brief Lack of Progress発生時の処理を行う
        """
        print("Handling Lack of Progress...")
        self._action.stop()

        while self._perception.isSwitchPressed():
            self._robot.update()
            time.sleep(0.1)

        print("Waiting for restart...")
        while not self._perception.isStartPressed():
            self._robot.update()
            time.sleep(0.05)

        print("Restarting from checkpoint...")
        self._navigator.recoverFromLoP()

    def _returnToStart(self) -> bool:
        """
        @brief スタート地点へ帰還する
        @return LoP が発生したらTrue
        """
        print("Returning to start position...")
        mapInstance = self._robot.mapInstance
        startPosition = (mapInstance.maxSize // 2, mapInstance.maxSize // 2)

        while mapInstance.currentPosition != startPosition:
            pathToStart = mapInstance.getPathTo(startPosition)

            if pathToStart is None:
                print("Cannot find path to start. Exploration may be incomplete.")
                return False

            print(f"Path to start: {[d.name for d in pathToStart]}")

            for direction in pathToStart:
                if self._perception.isSwitchPressed():
                    print("Toggle switch detected during return. Handling LoP.")
                    self._handleLackOfProgress()
                    return True

                isBlack, isLoP = self._navigator.moveTile(direction)

                if isLoP:
                    self._handleLackOfProgress()
                    return True

        print("Returned to start position.")
        return False

    def _goalPerformance(self) -> None:
        """
        @brief ゴール到達時の演出を行う
        """

        self._action.playMusic(buzzerSongs.matuken)
        self._action.flashLED(5, 1, color=[255, 255, 255])


