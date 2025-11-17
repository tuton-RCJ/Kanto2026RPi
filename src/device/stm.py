import serial
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
stmUART=STMUART(port)


class STS3032:
    def __init__(
        self,
    ):
        pass


class UnitV:
    def __init__(
        self,
    ):
        pass


class Loadcell:
    def __init__(
        self,
    ):
        pass


class Gyro:
    def __init__(
        self,
    ):
        pass


class Display:
    def __init__(
        self,
    ):
        pass


class Switch:
    def __init__(
        self,
    ):
        pass


class RescueKitServo:
    def __init__(
        self,
    ):
        pass



class STM:
    def __init__(
        self,
    ):
        self.sts3032: STS3032 = STS3032()
        self.unitv: UnitV = UnitV()
        self.loadcell: Loadcell = Loadcell()
        self.gyro: Gyro = Gyro()
        self.display: Display = Display()
        self.switch: Switch = Switch()
        self.rescuekitservo: RescueKitServo = RescueKitServo()

    def update(self):
        global stmUART
        uartData: UARTData = stmUART.read()

        # 各デバイスのデータを更新してね
        # ........
        pass
