# LiDAR、ToF、TSD10の距離センサの値をまとめてテストする
import time
from mazeUtils.device import stm

from mazeUtils.device import deviceEnums

def main():
    stmInstance = stm.STM()
    try:
        stmInstance.sts3032.stop()
        stmInstance.rescuekitservo.dropRescueKit(4,deviceEnums.Side.LEFT)
        time.sleep(3.5)
        stmInstance.rescuekitservo.dropRescueKit(4,deviceEnums.Side.RIGHT)
        time.sleep(3.5)
        for i in range(3):
            stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50})
            time.sleep(0.1)
            stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: -50, deviceEnums.Side.RIGHT: -50})
            time.sleep(0.1)
        stmInstance.sts3032.stop()

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
        stmInstance.sts3032.stop()
if __name__ == "__main__":
    main()