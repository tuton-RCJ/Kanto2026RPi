import time

import serial


class ArduinoNanoEveryUART:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0", # ttyACM0 : Arduino nano every, ttyUSB0 : 06-Display Board(CH340E)
        baudrate: int = 115200,
        timeout: float = 0.5,
    ):
        self._port = port
        self._serial = serial.Serial(
            port=self._port,
            baudrate=baudrate,
            timeout=0,
        )
        self._seq = 0
        self._timeout = timeout

    def _update_seq(self) -> None:
        self._seq = (self._seq + 1) % 256

    def _xor_check_digit(self, data: list[int]) -> int:
        check_digit = 0
        for value in data:
            check_digit ^= value
        return check_digit & 0xFF

    def _read_exact(self, size: int) -> bytes | None:
        start_time = time.time()
        buf = bytearray()
        while len(buf) < size:
            if time.time() - start_time > self._timeout:
                return None
            chunk = self._serial.read(size - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.001)
        return bytes(buf)

    def _flush_input(self) -> None:
        while self._serial.in_waiting:
            self._serial.read(self._serial.in_waiting)

    def request_tof_distance_mm(self) -> int | None:
        msg_type = 0
        self._update_seq()
        payload = [msg_type, self._seq]
        check_digit = self._xor_check_digit(payload)
        self._serial.write(bytes(payload + [check_digit]))

        response = self._read_exact(5)
        if response is None:
            return None
        if response[0] != msg_type or response[1] != self._seq:
            self._flush_input()
            return None

        expected_cd = self._xor_check_digit(list(response[0:4]))
        if expected_cd != response[4]:
            self._flush_input()
            return None

        return (response[2] << 8) | response[3]
    
    def camled(self, color: tuple[int, int, int]) -> bool:
        if any(c < 0 or c > 255 for c in color):
            print("Invalid color value for CamLED")
            return False
        msg_type = 4
        self._update_seq()
        payload = [msg_type, self._seq] + list(color)
        check_digit = self._xor_check_digit(payload)
        self._serial.write(bytes(payload + [check_digit]))

        response = self._read_exact(3)
        if response is None:
            return False
        if response[0] != msg_type or response[1] != self._seq:
            self._flush_input()
            return False

        expected_cd = self._xor_check_digit(list(response[0:2]))
        if expected_cd != response[2]:
            self._flush_input()
            return False

        return True
    
    def victimled(self, color: tuple[int, int, int]) -> bool:
        if any(c < 0 or c > 255 for c in color):
            print("Invalid color value for VictimLED")
            return False
        msg_type = 5
        self._update_seq()
        payload = [msg_type, self._seq] + list(color)
        check_digit = self._xor_check_digit(payload)
        self._serial.write(bytes(payload + [check_digit]))

        response = self._read_exact(3)
        if response is None:
            return False
        if response[0] != msg_type or response[1] != self._seq:
            self._flush_input()
            return False

        expected_cd = self._xor_check_digit(list(response[0:2]))
        if expected_cd != response[2]:
            self._flush_input()
            return False

        return True
    
    def update_oled(self, x_coord: int, y_coord: int, direction: int) -> bool:
        if not (0 <= x_coord <= 255 and 0 <= y_coord <= 255):
            return False
        if not (0 <= direction <= 3):
            return False

        msg_type = 1
        self._update_seq()
        payload = [msg_type, self._seq, x_coord, y_coord, direction]
        check_digit = self._xor_check_digit(payload)
        self._serial.write(bytes(payload + [check_digit]))

        response = self._read_exact(3)
        if response is None:
            return False
        if response[0] != msg_type or response[1] != self._seq:
            self._flush_input()
            return False

        expected_cd = self._xor_check_digit(list(response[0:2]))
        if expected_cd != response[2]:
            self._flush_input()
            return False

        return True
    
    def send_message(self,message:str)->bool:
        if len(message) > 60:
            print("Message too long for OLED display")
            return False
        
        msg_type = 2
        self._update_seq()
        data_length = len(message)
        payload = [msg_type, self._seq, data_length] + list(message.encode('utf-8'))
        check_digit = self._xor_check_digit(payload)
        self._serial.write(bytes(payload + [check_digit]))

        response = self._read_exact(3)
        if response is None:
            return False
        if response[0] != msg_type or response[1] != self._seq:
            self._flush_input()
            return False

        expected_cd = self._xor_check_digit(list(response[0:2]))
        if expected_cd != response[2]:
            self._flush_input()
            return False

        return True


class _NullArduinoNanoEveryUART:
    def __init__(self):
        pass
    def request_tof_distance_mm(self) -> int | None:
        return None

    def camled(self, color: tuple[int, int, int]) -> bool:
        return False

    def victimled(self, color: tuple[int, int, int]) -> bool:
        return False

    def update_oled(self, x_coord: int, y_coord: int, direction: int) -> bool:
        return False
    
    def send_message(self,message:str)->bool:
        return False