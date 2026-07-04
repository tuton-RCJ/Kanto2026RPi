# LiDAR、ToF、TSD10の距離センサの値をまとめてテストする
import time
from mazeUtils.device import stm

from mazeUtils.device import deviceEnums

def main():
    stmInstance = stm.STM()
    try:
        stmInstance.rescuekitservo.dropRescueKit(4,deviceEnums.Side.LEFT)
        time.sleep(4)
        stmInstance.rescuekitservo.dropRescueKit(4,deviceEnums.Side.RIGHT)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
        
if __name__ == "__main__":
    main()