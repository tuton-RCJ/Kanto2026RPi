from dataclasses import dataclass
import time
import struct

import serial

from . import deviceConstraints as deviceConst
from . import deviceEnums

class STMUART:
    def __init__(self, port: str = "/dev/ttyAMA0", timeout: float = 0.5):
        self._port = port
        self._serial = serial.Serial(
            port=self._port,
            baudrate=115200,
            # bytesize=serial.EIGHTBITS,
            # parity=serial.PARITY_NONE,
            # stopbits=serial.STOPBITS_ONE,
            # timeout=1,
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
        while self._serial.in_waiting < 22:
            if time.time() - start_time > self._timeout:
                print("STM UART timeout")
                return None

        data: bytes = self._serial.read(22)
        # データのチェック
        if data[0] == 0x00 and data[1] == self._seq:
            checkDigit = 0
            for b in data[0:21]:
                checkDigit ^= b
            if checkDigit == data[21]:
                return data[2:21]
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
        checkDigit = type.value[0] ^ self._seq
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
            checkDigit = type.value[0] ^ self._seq
            if checkDigit == response[2]:
                return True
        print("STM UART response error")
        return False

    def updateSeq(self):
        self._seq += 1
        self._seq = self._seq % 256


port: str = "/dev/ttyAMA0"
stmUART: STMUART = STMUART(port)


class STS3032:
    def __init__(
        self,
    ):
        pass

    def setMotorSpeed(self, motorSpeed: dict[deviceEnums.Side, int]) -> bool:
        """
        @brief モーターの速度を設定する
        @param motorSpeed: モーターの速度の辞書[Side, 速度(int,-100~100)]
        """
        global stmUART
        if not (-100 <= motorSpeed[deviceEnums.Side.LEFT] <= 100):
            print("invalid motor speed")
            return False
        if not (-100 <= motorSpeed[deviceEnums.Side.RIGHT] <= 100):
            print("invalid motor speed")
            return False

        data: bytes = bytes(
            [
                motorSpeed[deviceEnums.Side.LEFT] + 100,
                motorSpeed[deviceEnums.Side.RIGHT] + 100,
            ]
        )
        return stmUART.requestActuatorControl(
            deviceEnums.ActuatorControlType.STS_MOTOR, data
        )
    
    def turnRight(self, motorSpeed: int) -> bool:
        """
        @brief 右旋回する
        @param motorSpeed: モーターの速度(int,0~100)
        """
        global stmUART

        return self.setMotorSpeed({
            deviceEnums.Side.LEFT: motorSpeed,
            deviceEnums.Side.RIGHT: -motorSpeed,
        })
    
    def turnLeft(self, motorSpeed: int) -> bool:
        """
        @brief 左旋回する
        @param motorSpeed: モーターの速度(int,0~100)
        """
        global stmUART

        return self.setMotorSpeed({
            deviceEnums.Side.LEFT: -motorSpeed,
            deviceEnums.Side.RIGHT: motorSpeed,
        })
    
    def stop(self) -> bool:
        """
        @brief モーターを停止する
        """
        global stmUART

        return self.setMotorSpeed({
            deviceEnums.Side.LEFT: 0,
            deviceEnums.Side.RIGHT: 0,
        })


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
            deviceEnums.Side.LEFT: deviceEnums.UnitVStatus.NOTHING,
            deviceEnums.Side.RIGHT: deviceEnums.UnitVStatus.NOTHING,
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
            if self.minValue <= value < self.maxValue:
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

class ToF:
    def __init__(
        self,
    ):
        self.distance: list[float] = [-1 for _ in range(8)] # 時計回り
    
    def setDistance(self, distances: list[float]) -> bool:
        """
        @brief センサの距離をセットする
        @param distances: 距離のリスト[float(cm)]
        @return: 正常にセットできればTrue, エラーがあればFalse
        """
        # 今のところ 4 つしか tof ついてないので
        if len(distances) != 4:
            return False
        self.distance = distances
        return True
    
    def getDistance(self) -> list[float]:
        """
        @brief センサの距離を取得する
        @return: 距離のリスト[float(cm)]
        """
        return self.distance

@dataclass
class gyroData:
    """
    @brief ジャイロセンサのデータ構造体. X,Y,Z軸の角度を保持する.
    @note 単位はdeg
    """

    heading: float
    pitch: float
    roll: float


class Gyro:
    """
    @brief ジャイロセンサクラス
    @note readonly
    """

    def __init__(
        self,
    ):
        self.data: gyroData = gyroData(0.0, 0.0, 0.0)
        self.headingOffset: float = 0.0
        self.pitchOffset: float = 0.0
        self.rollOffset: float = 0.0
        pass

    def setValue(self, gyroData: gyroData):
        self.data = gyroData

    def setOffset(self, offset: gyroData):
        self.headingOffset = offset.heading
        self.pitchOffset = offset.pitch
        self.rollOffset = offset.roll

    def getValue(self) -> gyroData:

        res = gyroData(heading=self.data.heading, pitch=self.data.pitch, roll=self.data.roll)
        res.heading -= self.headingOffset
        res.pitch -= self.pitchOffset
        res.roll -= self.rollOffset
        res.heading %= 360
        res.pitch %= 360
        res.roll %= 360
        return res


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

    def dropRescueKit(self, num: int, side: deviceEnums.Side) -> bool:
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
        return stmUART.requestActuatorControl(deviceEnums.ActuatorControlType.RESCUE_KIT, data)


class LED:
    def __init__(
        self,
    ):
        pass

    def setColor(self, r: int, g: int, b: int) -> bool:
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
        self.tof: ToF = ToF()

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
            ## ロードセルでなくタッチセンサの値を取得している
            self.loadcell.setValue(
                {
                    deviceEnums.Side.LEFT: data[2] & (1<<7),
                    deviceEnums.Side.RIGHT: (data[2] & (1<<6))*2,   
                }
            )
            heading, pitch, roll = struct.unpack(">hhh", data[4:10])
            self.gyro.setValue(
                gyroData=gyroData(
                    heading=heading / 100.0,
                    pitch=-pitch / 100.0,
                    roll=-roll / 100.0,
                )
            )
            # data[7]の8bit目がプッシュスイッチ1の値、7bit目がトグルスイッチ1の値
            self.switch.setValue(
                pushSwitch1=bool((data[10] >> 7) & 0x01),
                toggleSwitch1=bool((data[10] >> 6) & 0x01),
            )
            #   int distance = ((int)sensorData[11 + i * 2] << 8) + (int)sensorData[12 + i * 2];
            # uart1.print(distance);
            # uart1.print(" ")
            self.tof.setDistance(
                [(data[11+i*2] << 8 | data[12+i*2])/10 for i in range(4)]
            )

            return True
