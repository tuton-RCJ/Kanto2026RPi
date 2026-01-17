"""
@brief センサ生の値を意味のある情報に変換するモジュール
"""

from . import mazeConstraints, mazeEnums
from .device import LiDAR, camera, deviceEnums

from .robot import Robot

class Perception:
    """
    @brief センサデータを解釈して意味のある情報を提供するクラス
    """

    def __init__(self, robotInstance: Robot) -> None:
        """
        @brief Perceptionクラスの初期化
        @param robotInstance Robotインスタンス
        """
        self._robot = robotInstance

    def getLidarPoints(self) -> list[LiDAR.Point] | None:
        """
        @brief 最新のLiDAR点群データを取得する
        @return LiDAR点群データのリスト。取得できない場合はNone
        """
        lidarWorker = self._robot.getLidarWorker()
        points, _ = lidarWorker.getLatest()
        return points

    def waitLidarPoints(self, timeoutSec: float = 1.0) -> list[LiDAR.Point]:
        """
        @brief LiDAR点群データを待機して取得する
        @param timeoutSec タイムアウト時間（秒）
        @return LiDAR点群データのリスト
        """
        lidarWorker = self._robot.getLidarWorker()
        return lidarWorker.waitFirst(timeoutSec)

    def detectTileColor(self) -> mazeEnums.tileType:
        """
        @brief カラーセンサとカメラを組み合わせてタイル色を判定する
        @return 検出されたタイルの種類
        """
        try:
            if not self._robot.colorSensorInstance.update():
                return mazeEnums.tileType.UNKNOWN

            r, g, b = self._robot.colorSensorInstance._colorRGB

            if (
                (mazeConstraints.BLACKTILE_RGB[1][0] < r < mazeConstraints.BLACKTILE_RGB[0][0])
                and (mazeConstraints.BLACKTILE_RGB[1][1] < g < mazeConstraints.BLACKTILE_RGB[0][1])
                and (mazeConstraints.BLACKTILE_RGB[1][2] < b < mazeConstraints.BLACKTILE_RGB[0][2])
            ):
                return mazeEnums.tileType.BLACK

            if (
                (mazeConstraints.BLUETILE_RGB[1][0] < r < mazeConstraints.BLUETILE_RGB[0][0])
                and (mazeConstraints.BLUETILE_RGB[1][1] < g < mazeConstraints.BLUETILE_RGB[0][1])
                and (mazeConstraints.BLUETILE_RGB[1][2] < b < mazeConstraints.BLUETILE_RGB[0][2])
            ):
                return mazeEnums.tileType.BLUE

            if (
                (mazeConstraints.REDTILE_RGB[1][0] < r < mazeConstraints.REDTILE_RGB[0][0])
                and (mazeConstraints.REDTILE_RGB[1][1] < g < mazeConstraints.REDTILE_RGB[0][1])
                and (mazeConstraints.REDTILE_RGB[1][2] < b < mazeConstraints.REDTILE_RGB[0][2])
            ):
                return mazeEnums.tileType.RED

            return mazeEnums.tileType.EMPTY

        except Exception as e:
            self._debugPrint(f"ColorSensor read failed: {e}")
            return mazeEnums.tileType.UNKNOWN

    def detectTileColorByCamera(self) -> bool:
        """
        @brief カメラを使用して黒タイルを検出する
        @return 黒タイルが検出されたら True
        """
        return camera.detectTileColor() == "BLACK"

    def isRampDetected(self) -> bool:
        """
        @brief ジャイロのRoll値から坂道を判定する
        @return 坂道が検出されたら True
        """
        rollValue = self._robot.stmInstance.gyro.getValue().roll
        minRoll = min(rollValue, 360 - rollValue)
        return 90 > minRoll > mazeConstraints.RAMP_DEG_THRESHOLD

    def getVictimInfo(self) -> dict[deviceEnums.Side, deviceEnums.UnitVStatus]:
        """
        @brief UnitVから被災者情報を取得する
        @return 各サイドの被災者種類を示す辞書
        @note 呼び出し元で robot.update() を実行しておく
        """
        return self._robot.stmInstance.unitv.getStatus()

    def isSwitchPressed(self) -> bool:
        """
        @brief トグルスイッチの状態を返す
        @return スイッチが押されていたら True
        """
        return self._robot.isToggleSwitchOn()

    def isStartPressed(self) -> bool:
        """
        @brief プッシュスイッチの状態を返す
        @return スイッチが押されていたら True
        """
        return self._robot.isPushSwitchOn()

    def isObstaclePressed(self) -> deviceEnums.Side | None:
        """
        @brief タッチセンサの状態を確認し、接触しているサイドを返す
        @return 接触しているサイド。接触していなければ None
        """
        pressed = self._robot.stmInstance.loadcell.getPressed()
        if pressed[deviceEnums.Side.LEFT]:
            return deviceEnums.Side.LEFT
        if pressed[deviceEnums.Side.RIGHT]:
            return deviceEnums.Side.RIGHT
        return None

    def isSilverTile(self) -> bool:
        """
        @brief フォトリフレクタで銀タイルを検出する
        @return 銀タイルが検出されたら True
        """
        return self._robot.photoReflectorInstance.isReflecting()

    def getHeading(self) -> float:
        """
        @brief 現在のジャイロのヘディング値を取得する
        @return ヘディング角度 (度)
        """
        return self._robot.stmInstance.gyro.getValue().heading

    def getRoll(self) -> float:
        """
        @brief 現在のジャイロのロール値を取得する
        @return ロール角度 (度)
        """
        return self._robot.stmInstance.gyro.getValue().roll

    def getTofDistance(self) -> list[float]:
        """
        @brief ToFセンサの距離を取得する
        @return 距離のリスト (cm)
        """
        return self._robot.stmInstance.tof.getDistance()

    def getUnitVLastUpdateTime(self) -> dict[deviceEnums.Side, int]:
        """
        @brief UnitVの最終更新時刻を取得する
        @return 各サイドの最終更新時刻
        """
        return self._robot.stmInstance.unitv.getLastUpdateTime()

    def getCertainAngleDist(
        self, angle: int | float | list[int | float], points: list[LiDAR.Point] | None = None
    ) -> int | list[int]:
        """
        @brief 指定角度のLiDAR距離を取得する
        @param angle 取得したい角度（度）
        @param points LiDAR の点群データ。 None の場合は最新を取得
        @return 距離 (cm)
        """
        if points is None:
            points = self.getLidarPoints()
        if points is None:
            return 1e9 if isinstance(angle, (int, float)) else [1e9] * len(angle)
        return LiDAR.getCertainAngleDist(angle, points)

    def isWallAheadTile(
        self, side: deviceEnums.Side, points: list[LiDAR.Point] | None = None
    ) -> bool:
        """
        @brief 指定サイドの前方に壁があるか判定する
        @param side 判定するサイド
        @param points LiDAR の点群データ。 None の場合は最新を取得
        @return 壁があれば True
        """
        if points is None:
            points = self.getLidarPoints()
        if points is None:
            return False
        return LiDAR.isWallAheadTile(points, side)

    def _debugPrint(self, *message: object) -> None:
        """
        @brief デバッグ出力
        @param message 出力メッセージ
        """
        if mazeConstraints.DEBUG_MODE:
            print(*message)
