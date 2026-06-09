import serial
import time
from config import get_logger

logger = get_logger(__name__)


class ColorSensor:
    """_summary_
    カラーセンサデバイスクラス
    Attributes:
        port (str): シリアルポート
    Methods:
        update() -> bool:
            センサの値を更新する
        setLEDColor(r: int, g: int, b: int) -> bool:
            LEDの色を設定する
    """

    def __init__(self, port: str = "/dev/ttyAMA5"):
        self.port = port

        self._serial = serial.Serial(port=self.port, baudrate=115200)

        self._colorRGB = (0, 0, 0)
        self._seq: int = 0
        self._timeout = 0.05

    def update(self) -> bool:
        return True
        data = self._requestSensorValues()
        if data is not None:
            self._colorRGB = (data[0], data[1], data[2])
            return True
        else:
            return False

    def _requestSensorValues(self) -> bytes | None:
        # センサの値を要求する
        self._serial.write(b"\x00")
        self._updateSeq()
        self._serial.write(bytes([self._seq]))
        checkDigit = 0 ^ self._seq
        self._serial.write(bytes([checkDigit]))

        # レスポンス待機
        start_time = time.time()
        while self._serial.in_waiting < 6:
            if time.time() - start_time > self._timeout:
                logger.warning("STM UART timeout")
                return None

        data: bytes = self._serial.read(6)
        for i in range(6):
            # print(f"data[{i}]: {data[i]}")
            pass
        # データのチェック
        if data[0] == 0x00 and data[1] == self._seq:
            checkDigit = 0
            for b in data[0:5]:
                checkDigit ^= b
            if checkDigit == data[5]:
                return data[2:5]
        logger.warning("STM UART data error")
        return None

    def setLEDColor(self, r: int, g: int, b: int):

        self._serial.write(bytes([0x01]))
        self._updateSeq()
        self._serial.write(bytes([self._seq]))
        self._serial.write(bytes([r]))
        self._serial.write(bytes([g]))
        self._serial.write(bytes([b]))
        checkDigit = 0x01 ^ self._seq
        for bb in [r, g, b]:
            checkDigit ^= bb
        self._serial.write(bytes([checkDigit]))

        # レスポンス待機
        start_time = time.time()
        while self._serial.in_waiting < 3:
            if time.time() - start_time > self._timeout:
                logger.warning("STM UART timeout")
                return False

        response: bytes = self._serial.read(3)

        # レスポンスのチェック
        if response[0] == 0x01 and response[1] == self._seq:
            checkDigit = 0x01 ^ self._seq
            if checkDigit == response[2]:
                return True
        logger.warning("STM UART response error")
        return False

    def _updateSeq(self):
        self._seq += 1
        if self._seq > 255:
            self._seq = 0
        return self._seq
