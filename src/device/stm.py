import serial
import device.deviceEnums as deviceEnums
import device.deviceConstrains as deviceConst
from dataclasses import dataclass


@dataclass
class UARTData:
    pass


class STMUART:
    def __init__(self, port="/dev/ttyAMA4"):
        self.port = port

    def read(self) -> UARTData:
        # シリアル通信でSTMからデータを読む
        return UARTData()


port = "/dev/ttyAMA4"
stmUART = STMUART(port)


class STS3032:
    def __init__(
        self,
    ):
        pass

    def setMotorSpeed(self, motorSpeed: dict[deviceEnums.Side, int]):
        """
        @brief モーターの速度を設定する
        @param motorSpeed: モーターの速度の辞書[Side, 速度(int,0~100)]
        """
        pass

    # def setEncoderValue(self, encoderValue: dict[deviceEnums.Side, int]):
    #     """
    #     @brief エンコーダの値を設定する
    #     @param encoderValue: エンコーダの値の辞書[Side, 値(int)]
    #     """
    #     pass


class UnitV:
    def __init__(
        self,
    ):
        self.status: dict[deviceEnums.Side, deviceEnums.UnitVStatus] = {
            deviceEnums.Side.LEFT: deviceEnums.UnitVStatus.UNKNOWN,
            deviceEnums.Side.RIGHT: deviceEnums.UnitVStatus.UNKNOWN,
        }

    def setStatus(self, status: dict[deviceEnums.Side, deviceEnums.UnitVStatus]):
        """
        @brief UnitVのステータスを設定する
        @param status: ステータスの辞書[Side, UnitVStatus]
        """
        self.status = status

    def getStatus(self) -> dict[deviceEnums.Side, deviceEnums.UnitVStatus]:
        """
        @brief UnitVのステータスを取得する
        @return: ステータスの辞書[Side, UnitVStatus]
        """
        return self.status


class Loadcell:
    def __init__(
        self,
    ):
        self.raw: dict[deviceEnums.Side, int] = {
            deviceEnums.Side.LEFT: 0,
            deviceEnums.Side.RIGHT: 0,
        }
        self.pressed: dict[deviceEnums.Side, bool] = {
            deviceEnums.Side.LEFT: False,
            deviceEnums.Side.RIGHT: False,
        }
        self.THRESHOULD = deviceConst.LOADCELL_THRESHOULD
        self.minValue = 0
        self.maxValue = 255
        pass

    def setValue(self, setData: dict[deviceEnums.Side, int]) -> bool:
        """
        @brief センサの値をセットする
        @param setData: センサの値の辞書[Side, ADC値]
        @return: 正常にセットできればTrue, エラーがあればFalse
        """
        error = False
        for side, value in setData.items():
            self.raw[side] = value
            if self.minValue < value < self.maxValue:
                if value > self.THRESHOULD:
                    self.pressed[side] = True
                else:
                    self.pressed[side] = False
            else:
                error = True
        return not error

    def getRawValue(self) -> dict[deviceEnums.Side, int]:
        """
        @brief センサの生の値を取得する
        @return: センサの値の辞書[Side, ADC値(int)]
        """
        return self.raw

    def getPressed(self) -> dict[deviceEnums.Side, bool]:
        """
        @brief センサが押されているかを取得する
        @return: センサが押されているかの辞書[Side, 押されているか]
        """
        return self.pressed


@dataclass
class gyroData:
    """
    @brief ジャイロセンサのデータ構造体. X,Y,Z軸の角度を保持する.
    @note 単位はdeg
    """

    x: int
    y: int
    z: int


class Gyro:
    """
    @brief ジャイロセンサクラス
    @note readonly
    """

    def __init__(
        self,
    ):
        self.data: gyroData = gyroData(0, 0, 0)
        pass

    def setValue(self, gyroData):
        self.data = gyroData

    def getValue(self) -> gyroData:
        return self.data


# class Display:
#     """
#     @brief ディスプレイクラス。たぶん変える
#     """
#     def __init__(
#         self,
#     ):
#         pass


class Switch:
    """
    @brief
    """

    def __init__(
        self,
    ):
        self.pushSwitch1 = False
        self.toggleSwitch1 = False

    def setValue(self, pushSwitch1: bool, toggleSwitch1: bool):
        self.pushSwitch1 = pushSwitch1
        self.toggleSwitch1 = toggleSwitch1

    def getPushSwitch1(self) -> bool:
        return self.pushSwitch1
    def getToggleSwitch1(self) -> bool:
        return self.toggleSwitch1


class RescueKitServo:
    """
    @brief レスキューキット
    @note writeonly
    """

    def __init__(
        self,
    ):
        pass

    def dropRescueKit(self, num: int, side: deviceEnums.Side):
        """
        @brief レスキューキットを落とす
        @param num: レスキューキットの数
        @param side: レスキューキットを落とすサイド
        """
        pass


class STM:
    def __init__(
        self,
    ):
        self.sts3032: STS3032 = STS3032()
        self.unitv: UnitV = UnitV()
        self.loadcell: Loadcell = Loadcell()
        self.gyro: Gyro = Gyro()
        # self.display: Display = Display()
        self.switch: Switch = Switch()
        self.rescuekitservo: RescueKitServo = RescueKitServo()

    def update(self):
        global stmUART
        uartData: UARTData = stmUART.read()

        # 各デバイスのデータを更新してね
        # ........
        pass
