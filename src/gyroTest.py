import time
from mazeUtils.device import stm


def main():
    stmInstance = stm.STM()
    try:
        while True:
            stmInstance.update()
            print(stmInstance.gyro.getValue().heading, stmInstance.gyro.getValue().pitch, stmInstance.gyro.getValue().roll)
    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    
    
if __name__ == "__main__":
    main()
