# LiDAR、ToF、TSD10の距離センサの値をまとめてテストする
import time
from mazeUtils.device import stm
from mazeUtils.device import LiDAR


def main():
    stmInstance = stm.STM()
    lidar = LiDAR.initializeLidar()
    try:
        
        print("データ読み取り中... (Ctrl+C で終了)")
        while True:
            s=input()
            # 受信データをチェックし、更新があれば表示
            stmInstance.update()
            stmInstance.frontTSD10.update()
            points = LiDAR.getLiDARScan(lidar)
            dist = LiDAR.getCertainAngleDist(0, points)
            print(f"LiDAR前方距離: {dist} cm")
            print(f"TSD10距離: {stmInstance.frontTSD10.get_distance() / 10} cm")
            print(f"ToF距離: {stmInstance.tof.getDistance()} mm")
            # CPU負荷を下げるためのわずかなウェイト
            # time.sleep(2)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        print("センサーを停止します...")
        stmInstance.frontTSD10.stop()
        stmInstance.frontTSD10.close()
        
if __name__ == "__main__":
    main()