from dataclasses import dataclass
import time
import struct
from typing import Optional
import serial

from . import deviceConstraints as deviceConst
from . import deviceEnums
from .buzzerSongs import MusicData, detectedVictim


from . import buzzerSongs


class STMUART:
    def __init__(self, port: str, timeout: float = 0.5):
        self._port = port
        self._serial: Optional[serial.Serial] = None
        self._seq: int = 0
        self._timeout = timeout
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=115200,
                # bytesize=serial.EIGHTBITS,
                # parity=serial.PARITY_NONE,
                # stopbits=serial.STOPBITS_ONE,
                # timeout=1,
            )
        except serial.SerialException as e:
            print(
                "[STM UART] Serial open failed: "
                f"port={self._port} baudrate=115200 error={e}"
            )
        except Exception as e:
            print(
                "[STM UART] Unexpected serial init error: "
                f"port={self._port} error={type(e).__name__}: {e}"
            )

    def read(self) -> bytes:
        # シリアル通信でSTMからデータを読む
        if self._serial is None:
            print(f"[STM UART] Read failed: serial is not initialized (port={self._port})")
            return b""
        try:
            return self._serial.read(1)
        except serial.SerialException as e:
            print(f"[STM UART] Read failed: port={self._port} error={e}")
            return b""

    def write(self, data: bytes):
        # シリアル通信でSTMにデータを書く
        if self._serial is None:
            print(
                f"[STM UART] Write failed: serial is not initialized (port={self._port})"
            )
            return
        try:
            self._serial.write(data)
        except serial.SerialException as e:
            print(
                "[STM UART] Write failed: "
                f"port={self._port} size={len(data)}B error={e}"
            )

    def requestSensorValues(self) -> bytes | None:
        if self._serial is None:
            print(
                f"[STM UART][Sensor] Request failed: serial is not initialized (port={self._port})"
            )
            return None

        # センサの値を要求する
        try:
            # 前回通信の残骸があるとフレーム先頭がずれるため、要求前に受信バッファをクリアする
            self._serial.reset_input_buffer()

            self._serial.write(b"\x00")
            self.updateSeq()
            self._serial.write(bytes([self._seq]))
            checkDigit = 0 ^ self._seq
            self._serial.write(bytes([checkDigit]))

            # ストリームからフレームを探索して同期回復する
            start_time = time.time()
            frame_len = 24
            buffer = bytearray()
            dropped_bytes = 0

            while time.time() - start_time <= self._timeout:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    buffer.extend(self._serial.read(waiting))

                while len(buffer) >= frame_len:
                    # 先頭が期待ヘッダでなければ1byteずつ捨てて再同期する
                    if buffer[0] != 0x00 or buffer[1] != self._seq:
                        dropped_bytes += 1
                        del buffer[0]
                        continue

                    frame = bytes(buffer[:frame_len])
                    checksum = 0
                    for b in frame[0:23]:
                        checksum ^= b

                    if checksum != frame[23]:
                        print(
                            "[STM UART][Sensor] Checksum mismatch while resync: "
                            f"expected=0x{checksum:02X} actual=0x{frame[23]:02X} "
                            f"seq={self._seq} candidate={frame.hex(' ')}"
                        )
                        dropped_bytes += 1
                        del buffer[0]
                        continue

                    if dropped_bytes > 0:
                        print(
                            "[STM UART][Sensor] Frame resynchronized: "
                            f"dropped={dropped_bytes}B seq={self._seq}"
                        )

                    del buffer[:frame_len]
                    return frame[2:23]

                time.sleep(0.001)

            print(
                "[STM UART][Sensor] Timeout while searching valid frame: "
                f"elapsed={time.time() - start_time:.3f}s timeout={self._timeout:.3f}s "
                f"seq={self._seq} buffered={len(buffer)}B dropped={dropped_bytes}B "
                f"tail={bytes(buffer[-24:]).hex(' ')}"
            )
            return None
        except serial.SerialException as e:
            print(
                "[STM UART][Sensor] Serial error during transaction: "
                f"port={self._port} seq={self._seq} error={e}"
            )
            return None
        except Exception as e:
            print(
                "[STM UART][Sensor] Unexpected exception: "
                f"port={self._port} seq={self._seq} error={type(e).__name__}: {e}"
            )
            return None

    def requestActuatorControl(
        self, type: deviceEnums.ActuatorControlType, data: bytes
    ) -> bool:
        if self._serial is None:
            print(
                "[STM UART][Actuator] Request failed: "
                f"type={type.name} serial is not initialized (port={self._port})"
            )
            return False

        # データ長のチェック
        expected_len = type.dataLength()
        if expected_len is not None and expected_len >= 0 and len(data) != expected_len:
            print(
                "[STM UART][Actuator] Data length mismatch: "
                f"type={type.name} expected={expected_len}B actual={len(data)}B "
                f"data={data.hex(' ')}"
            )
            return False

        try:
            self._serial.write(bytes(type.value))
            self.updateSeq()
            self._serial.write(bytes([self._seq]))
            for b in data:
                self._serial.write(bytes([b]))
            checkDigit = type.value[0] ^ self._seq
            for b in data:
                checkDigit ^= b
            self._serial.write(bytes([checkDigit]))

            # ストリームからACKフレームを探索して同期回復する
            start_time = time.time()
            ack_len = 3
            buffer = bytearray()
            dropped_bytes = 0

            while time.time() - start_time <= self._timeout:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    buffer.extend(self._serial.read(waiting))

                while len(buffer) >= ack_len:
                    # 先頭が期待ACKヘッダでなければ1byteずつ捨てて再同期する
                    if buffer[0] != type.value[0] or buffer[1] != self._seq:
                        dropped_bytes += 1
                        del buffer[0]
                        continue

                    response = bytes(buffer[:ack_len])
                    checksum = type.value[0] ^ self._seq
                    if checksum != response[2]:
                        print(
                            "[STM UART][Actuator] Checksum mismatch while resync: "
                            f"type={type.name} expected=0x{checksum:02X} actual=0x{response[2]:02X} "
                            f"seq={self._seq} candidate={response.hex(' ')}"
                        )
                        dropped_bytes += 1
                        del buffer[0]
                        continue

                    if dropped_bytes > 0:
                        print(
                            "[STM UART][Actuator] ACK resynchronized: "
                            f"type={type.name} dropped={dropped_bytes}B seq={self._seq}"
                        )

                    del buffer[:ack_len]
                    return True

                time.sleep(0.001)

            print(
                "[STM UART][Actuator] Timeout while searching valid ACK: "
                f"type={type.name} elapsed={time.time() - start_time:.3f}s timeout={self._timeout:.3f}s "
                f"seq={self._seq} payload={data.hex(' ')} buffered={len(buffer)}B dropped={dropped_bytes}B "
                f"tail={bytes(buffer[-24:]).hex(' ')}"
            )
            return False
        except serial.SerialException as e:
            print(
                "[STM UART][Actuator] Serial error during transaction: "
                f"type={type.name} port={self._port} seq={self._seq} error={e}"
            )
            return False
        except Exception as e:
            print(
                "[STM UART][Actuator] Unexpected exception: "
                f"type={type.name} port={self._port} seq={self._seq} "
                f"error={e.__class__.__name__}: {e}"
            )
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
            print(
                "[STS3032] Invalid motor speed: "
                f"side=LEFT value={motorSpeed[deviceEnums.Side.LEFT]} range=[-100, 100]"
            )
            return False
        if not (-100 <= motorSpeed[deviceEnums.Side.RIGHT] <= 100):
            print(
                "[STS3032] Invalid motor speed: "
                f"side=RIGHT value={motorSpeed[deviceEnums.Side.RIGHT]} range=[-100, 100]"
            )
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

        return self.setMotorSpeed(
            {
                deviceEnums.Side.LEFT: motorSpeed,
                deviceEnums.Side.RIGHT: -motorSpeed,
            }
        )

    def turnLeft(self, motorSpeed: int) -> bool:
        """
        @brief 左旋回する
        @param motorSpeed: モーターの速度(int,0~100)
        """
        global stmUART

        return self.setMotorSpeed(
            {
                deviceEnums.Side.LEFT: -motorSpeed,
                deviceEnums.Side.RIGHT: motorSpeed,
            }
        )

    def stop(self) -> bool:
        """
        @brief モーターを停止する
        """
        global stmUART

        return self.setMotorSpeed(
            {
                deviceEnums.Side.LEFT: 0,
                deviceEnums.Side.RIGHT: 0,
            }
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
            deviceEnums.Side.LEFT: deviceEnums.UnitVStatus.NOTHING,
            deviceEnums.Side.RIGHT: deviceEnums.UnitVStatus.NOTHING,
        }
        self.lastUpdateTime: dict[deviceEnums.Side, Optional[int]] = {
            deviceEnums.Side.LEFT: None,
            deviceEnums.Side.RIGHT: None,
        }

    def setStatus(
        self,
        status: dict[deviceEnums.Side, deviceEnums.UnitVStatus],
        updateTime: dict[deviceEnums.Side, Optional[int]],
    ):
        """
        @brief UnitVのステータスを設定する
        @param status: ステータスの辞書[Side, UnitVStatus]
        """
        self.status = status.copy()
        self.lastUpdateTime = updateTime.copy()

    def getStatus(self) -> dict[deviceEnums.Side, deviceEnums.UnitVStatus]:
        """
        @brief UnitVのステータスを取得する
        @return: ステータスの辞書[Side, UnitVStatus]
        """
        return self.status

    def getLastUpdateTime(self) -> dict[deviceEnums.Side, Optional[int]]:
        """
        @brief UnitVのステータスの最終更新時間を取得する
        @return: 最終更新時間の辞書[Side, 時間(ms)]
        """
        return self.lastUpdateTime


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
                print(
                    "[Loadcell] Value out of range: "
                    f"side={side.name} value={value} range=[{self.minValue}, {self.maxValue})"
                )
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
        self.distance: list[float] = [-1 for _ in range(8)]  # 時計回り

    def setDistance(self, distances: list[float]) -> bool:
        """
        @brief センサの距離をセットする
        @param distances: 距離のリスト[float(cm)]
        @return: 正常にセットできればTrue, エラーがあればFalse
        """
        # 今のところ 4 つしか tof ついてないので
        if len(distances) != 4:
            print(
                "[ToF] Invalid distance list length: "
                f"expected=4 actual={len(distances)} values={distances}"
            )
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

    def setValue(self, gyrodata: gyroData):
        self.data = gyroData(
            heading=gyrodata.heading, pitch=gyrodata.pitch, roll=gyrodata.roll
        )

    def setOffset(self, offset: gyroData):
        self.headingOffset = offset.heading
        self.pitchOffset = offset.pitch
        self.rollOffset = offset.roll

    def getValue(self) -> gyroData:

        res = gyroData(
            heading=self.data.heading, pitch=self.data.pitch, roll=self.data.roll
        )
        res.heading -= self.headingOffset
        res.pitch -= self.pitchOffset
        res.roll -= self.rollOffset
        res.heading %= 360
        res.pitch %= 360
        res.roll %= 360
        return res



class Buzzer:
    """
    @brief Buzzerのクラス。
    """

    def __init__(
        self,
    ):
        pass

    def playMusic(self, music: MusicData) -> bool:
        """
        @brief 音楽を再生する
        @param music: 再生する音楽データ
        @return: 成功したらTrue、失敗したらFalse
        """
        global stmUART
        # データの作成
        # 音符数(1byte) + 各音符(周波数2byte, 長さ2byte)
        data: bytes = bytes([len(music.notes)])
        for note in music.notes:
            freq = note[0]
            length = note[1]
            data += struct.pack(">HH", freq, length)
        return stmUART.requestActuatorControl(
            deviceEnums.ActuatorControlType.BUZZER, data
        )


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
        return stmUART.requestActuatorControl(
            deviceEnums.ActuatorControlType.RESCUE_KIT, data
        )


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
            print(f"[LED] Invalid color value: channel=R value={r} range=[0, 255]")
            return False
        if not (0 <= g <= 255):
            print(f"[LED] Invalid color value: channel=G value={g} range=[0, 255]")
            return False
        if not (0 <= b <= 255):
            print(f"[LED] Invalid color value: channel=B value={b} range=[0, 255]")
            return False

        data: bytes = bytes(
            [
                r,
                g,
                b,
            ]
        )
        return stmUART.requestActuatorControl(deviceEnums.ActuatorControlType.LED, data)


class CamLED:
    def __init__(
        self,
    ):
        pass

    def setColor(self, r: int, g: int, b: int) -> bool:
        """
        @brief カメラ用LEDの色を設定する
        @param r: 赤の値(0~255)
        @param g: 緑の値(0~255)
        @param b: 青の値(0~255)
        """
        global stmUART
        # 値の範囲チェック
        if not (0 <= r <= 255):
            print(f"[CamLED] Invalid color value: channel=R value={r} range=[0, 255]")
            return False
        if not (0 <= g <= 255):
            print(f"[CamLED] Invalid color value: channel=G value={g} range=[0, 255]")
            return False
        if not (0 <= b <= 255):
            print(f"[CamLED] Invalid color value: channel=B value={b} range=[0, 255]")
            return False

        data: bytes = bytes(
            [
                r,
                g,
                b,
            ]
        )
        return stmUART.requestActuatorControl(
            deviceEnums.ActuatorControlType.CAMLED, data
        )


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
        # self.camled: CamLED = CamLED()
        self.tof: ToF = ToF()
        self.buzzer: Buzzer = Buzzer()

    def update(self) -> bool:
        global stmUART

        data = stmUART.requestSensorValues()
        if data is None:
            print("[STM] update failed: requestSensorValues returned None")
            return False
        else:
            if len(data) != 21:
                print(
                    "[STM] Sensor payload length mismatch: "
                    f"expected=21B actual={len(data)}B raw={data.hex(' ')}"
                )
                return False
            try:
                self.unitv.setStatus(
                    {
                        deviceEnums.Side.LEFT: deviceEnums.UnitVStatus(data[0]),
                        deviceEnums.Side.RIGHT: deviceEnums.UnitVStatus(data[1]),
                    },
                    {
                        deviceEnums.Side.LEFT: data[2],
                        deviceEnums.Side.RIGHT: data[3],
                    },
                )
                ## ロードセルでなくタッチセンサの値を取得している
                self.loadcell.setValue(
                    {
                        deviceEnums.Side.LEFT: data[4] & (1 << 7),
                        deviceEnums.Side.RIGHT: (data[4] & (1 << 6)) * 2,
                    }
                )
                heading, pitch, roll = struct.unpack(">HHH", data[6:12])
                self.gyro.setValue(
                    gyrodata=gyroData(
                        heading=heading / 100.0,
                        pitch=-pitch / 100.0,
                        roll=-roll / 100.0,
                    )
                )
                # data[7]の8bit目がプッシュスイッチ1の値、7bit目がトグルスイッチ1の値
                self.switch.setValue(
                    pushSwitch1=bool((data[12] >> 7) & 0x01),
                    toggleSwitch1=bool((data[12] >> 6) & 0x01),
                )
                #   int distance = ((int)sensorData[11 + i * 2] << 8) + (int)sensorData[12 + i * 2];
                # uart1.print(distance);
                # uart1.print(" ")
                tof_ok = self.tof.setDistance(
                    [
                        (
                            ((data[13 + i * 2] << 8 | data[14 + i * 2]) / 10)
                            if ((data[13 + i * 2] << 8 | data[14 + i * 2]) != 0)
                            else self.tof.getDistance()[i]
                        )
                        for i in range(4)
                    ]
                )
                if not tof_ok:
                    print("[STM] update warning: failed to update ToF distances")
            except ValueError as e:
                print(
                    "[STM] update failed: invalid enum/field value in sensor payload "
                    f"error={e} raw={data.hex(' ')}"
                )
                return False
            except struct.error as e:
                print(
                    "[STM] update failed: struct unpack error "
                    f"error={e} raw={data.hex(' ')}"
                )
                return False
            except Exception as e:
                print(
                    "[STM] update failed: unexpected parse error "
                    f"error={type(e).__name__}: {e} raw={data.hex(' ')}"
                )
                return False

            return True
