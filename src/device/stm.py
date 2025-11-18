import serial
import device.deviceEnums as deviceEnums
import device.deviceConstrains as deviceConst
from dataclasses import dataclass
import time


class STMUART:
    def __init__(self, port: str = "/dev/ttyAMA4", timeout: float = 0.5):
        self._port = port
        self._serial = serial.Serial(
            port=self._port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )
        self._seq: int = 0
        self._timeout = timeout

    def read(self) -> bytes:
        # シリアル通信でSTMからデータを読む
        return self._serial.read(1)

    def write(self, data: bytes):
        # シリアル通信でSTMにデータを書く
        self._serial.write(data)

    def requestSensorValues(self) -> bytes | None:
        # センサの値を要求する
        self._serial.write(b"\x00")
        self.updateSeq()
        self._serial.write(bytes([self._seq]))
        checkDigit = 0 ^ self._seq
        self._serial.write(bytes([checkDigit]))

        # レスポンス待機
        start_time = time.time()
        while self._serial.in_waiting < 11:
            if time.time() - start_time > self._timeout:
                print("STM UART timeout")
                return None

        data: bytes = self._serial.read(11)
        # データのチェック
        if data[0] == 0x00 and data[1] == self._seq:
            checkDigit = 0
            for b in data[0:10]:
                checkDigit ^= b
            if checkDigit == data[10]:
                return data[2:10]
        print("STM UART data error")
        return None

    def requestActuatorControl(
        self, type: deviceEnums.ActuatorControlType, data: bytes
    ) -> bool:

        # データ長のチェック
        if len(data) != type.dataLength():
            print("Actuator Control data length error")
            return False

        self._serial.write(bytes(type.value))
        self.updateSeq()
        self._serial.write(bytes([self._seq]))
        for b in data:
            self._serial.write(bytes([b]))
        checkDigit = 0 ^ self._seq
        for b in data:
            checkDigit ^= b
        self._serial.write(bytes([checkDigit]))

        # レスポンス待機
        start_time = time.time()
        while self._serial.in_waiting < 3:
            if time.time() - start_time > self._timeout:
                print("STM UART timeout")
                return False

        response: bytes = self._serial.read(3)

        # レスポンスのチェック
        if response[0] == type.value[0] and response[1] == self._seq:
            checkDigit = 0 ^ self._seq
            if checkDigit == response[2]:
                return True
        print("STM UART response error")
        return False

    def updateSeq(self):
        self._seq += 1
        self._seq = self._seq % 256


port: str = "/dev/ttyAMA4"
stmUART: STMUART = STMUART(port)


class STS3032:
    def __init__(
        self,
    ):
        pass

    def setMotorSpeed(self, motorSpeed: dict[deviceEnums.Side, int]) -> bool:
        """
        @brief モーターの速度を設定する
        @param motorSpeed: モーターの速度の辞書[Side, 速度(int,0~100)]
        """
        global stmUART
        if not (0 <= motorSpeed[deviceEnums.Side.LEFT] <= 100):
            print("invalid motor speed")
            return False
        if not (0 <= motorSpeed[deviceEnums.Side.RIGHT] <= 100):
            print("invalid motor speed")
            return False

        data: bytes = bytes(
            [
                motorSpeed[deviceEnums.Side.LEFT],
                motorSpeed[deviceEnums.Side.RIGHT],
            ]
        )
        return stmUART.requestActuatorControl(
            deviceEnums.ActuatorControlType.STS_MOTOR, data
        )

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

    heading: int
    pitch: int
    roll: int


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

    def setValue(self, gyroData: gyroData):
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
        global stmUART

        data: bytes = bytes(
            [
                side.value,
                num,
            ]
        )
        stmUART.requestActuatorControl(deviceEnums.ActuatorControlType.RESCUE_KIT, data)

        pass


class LED:
    def __init__(
        self,
    ):
        pass

    def setLEDColor(self, r: int, g: int, b: int) -> bool:
        """
        @brief LEDの色を設定する
        @param r: 赤の値(0~255)
        @param g: 緑の値(0~255)
        @param b: 青の値(0~255)
        """
        global stmUART
        # 値の範囲チェック
        if not (0 <= r <= 255):
            print("invalid LED color value")
            return False
        if not (0 <= g <= 255):
            print("invalid LED color value")
            return False
        if not (0 <= b <= 255):
            print("invalid LED color value")
            return False

        data: bytes = bytes(
            [
                r,
                g,
                b,
            ]
        )
        return stmUART.requestActuatorControl(deviceEnums.ActuatorControlType.LED, data)


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
        self.led: LED = LED()

    def update(self) -> bool:
        global stmUART

        data = stmUART.requestSensorValues()
        if data is None:
            return False
        else:
            self.unitv.setStatus(
                {
                    deviceEnums.Side.LEFT: deviceEnums.UnitVStatus(data[0]),
                    deviceEnums.Side.RIGHT: deviceEnums.UnitVStatus(data[1]),
                }
            )
            self.loadcell.setValue(
                {
                    deviceEnums.Side.LEFT: data[2],
                    deviceEnums.Side.RIGHT: data[3],
                }
            )
            # ジャイロの値は360度を1/2に圧縮して送信されるので、2倍にして元に戻す
            self.gyro.setValue(gyroData(data[4] * 2, data[5] * 2, data[6] * 2))
            # data[7]の8bit目がプッシュスイッチ1の値、7bit目がトグルスイッチ1の値
            self.switch.setValue(
                pushSwitch1=bool((data[7] >> 7) & 0x01),
                toggleSwitch1=bool((data[7] >> 6) & 0x01),
            )
            return True
