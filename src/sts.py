from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import deviceConstraints, stm
from mazeUtils.device import LiDAR
from mazeUtils.device import arduinoNanoEvery
from mazeUtils.device import colorsensor
from mazeUtils.device import deviceEnums
from mazeUtils.device import buzzerSongs
from mazeUtils.device import camera
from mazeUtils.device import photoReflector
from mazeUtils.device import tsd10
from mazeUtils import mazeEnums
import math
from mazeUtils import mazeConstraints
import ydlidar
import time
import math

MOVE_1cm_TIME = 0.050

def main():
    # USBシリアル変換モジュールの場合、通常 '/dev/ttyUSB0' や '/dev/ttyACM0' になります。
    # ボーレートはTSD10の既定値（不明な場合は通常 9600 や 115200 など）に合わせ変化させてください。
    sensor = tsd10.TSD10(port='/dev/ttyUSB0', baudrate=460800)
    lidar = LiDAR.initializeLidar()
    stmInstance = stm.STM()
    
    try:
        stmInstance.sts3032.setMotorSpeed({deviceEnums.Side.LEFT: 70, deviceEnums.Side.RIGHT: 70})
        time.sleep(30)
        stmInstance.sts3032.stop()
        # while True:
        #     stmInstance.frontTSD10.update()
        #     print(f"TSD10 distance: {stmInstance.frontTSD10.get_distance() / 10} cm")
        #     time.sleep(1)
            
        print("センサーを開始します...")
        sensor.start()
        
        print("データ読み取り中... (Ctrl+C で終了)")
        while True:
            s=input()
            # 受信データをチェックし、更新があれば表示
            if sensor.update():
                distance = sensor.get_distance()
                if distance == 65535:
                    print("測定範囲外エラー")
                else:
                    print(f"距離: {distance} mm")
            points = LiDAR.getLiDARScan(lidar)
            dist = LiDAR.getCertainAngleDist(0, points)
            print(f"LiDAR前方距離: {dist} cm")
            # CPU負荷を下げるためのわずかなウェイト
            # time.sleep(2)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        print("センサーを停止します...")
        sensor.stop()
        sensor.close()
        stmInstance.sts3032.stop()
        

# def main():
#     stmInstance = stm.STM()
#     print(stmInstance.gyro.headingOffset, stmInstance.gyro.rollOffset, stmInstance.gyro.pitchOffset)
#     try:
#         while True:
#             stmInstance.update()
#             print(
#                 stmInstance.gyro.getValue().heading,
#                 stmInstance.gyro.getValue().roll,
#                 stmInstance.gyro.getValue().pitch,
#             )
#             print(
#                 stmInstance.gyro.getValue().heading,
#                 stmInstance.gyro.getValue().roll,
#                 stmInstance.gyro.getValue().pitch,
#             )
#             time.sleep(0.2)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         return
#     nowDist = 0
#     lastTime = time.time()
#     nowhight = 0
#     stmInstance.update()
#     print(
#         stmInstance.gyro.getValue().heading,
#         stmInstance.gyro.getValue().roll,
#         stmInstance.gyro.getValue().pitch,
#     )
#     time.sleep(3)
#     lastToFDist = stmInstance.tof.getDistance()[2]
#     try:
#         while stmInstance.gyro.getValue().roll < 5:

#             stmInstance.update()
#             movedDist = (time.time() - lastTime) / MOVE_1cm_TIME
#             # movedDist = stmInstance.tof.getDistance()[2] - lastToFDist
#             # lastToFDist = stmInstance.tof.getDistance()[2]
#             lastTime = time.time()
#             nowDist += movedDist * math.cos(
#                 math.radians(stmInstance.gyro.getValue().roll)
#             )
#             nowhight += movedDist * math.sin(
#                 math.radians(stmInstance.gyro.getValue().roll)
#             )
#             print(nowDist)

#             # if nowDist >= 30:
#             #     stmInstance.sts3032.stop()
#             #     nowDist = 0
#             #     print("hight", nowhight, "cm")
#             #     time.sleep(1)
#             #     lastTime = time.time()

#             stmInstance.sts3032.setMotorSpeed(
#                 {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50}
#             )
#             time.sleep(0.01)
#             # while True:
#             #     s = input()
#             #     stmInstance.update()
#             #     print(
#             #         stmInstance.gyro.getValue().heading,
#             #         stmInstance.gyro.getValue().roll,
#             #         stmInstance.gyro.getValue().pitch,
#             #     )
#         stmInstance.sts3032.stop()
#         time.sleep(2)
        
#         nowDist = 0
#         nowhight = 0
#         lastTime=time.time()
#         while stmInstance.gyro.getValue().roll > 3:

#             stmInstance.update()
#             movedDist = (time.time() - lastTime) / MOVE_1cm_TIME
#             # movedDist = stmInstance.tof.getDistance()[2] - lastToFDist
#             # lastToFDist = stmInstance.tof.getDistance()[2]
#             lastTime = time.time()
#             nowDist += movedDist * math.cos(
#                 math.radians(stmInstance.gyro.getValue().roll)
#             )
#             nowhight += movedDist * math.sin(
#                 math.radians(stmInstance.gyro.getValue().roll)
#             )
#             print(nowDist)

#             # if nowDist >= 30:
#             #     stmInstance.sts3032.stop()
#             #     nowDist = 0
#             #     print("hight", nowhight, "cm")
#             #     time.sleep(1)
#             #     lastTime = time.time()

#             stmInstance.sts3032.setMotorSpeed(
#                 {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50}
#             )
#             time.sleep(0.01)
#         stmInstance.sts3032.stop()
#         print("hight", nowhight, "cm")
#         print("dist", nowDist, "cm")
#     except KeyboardInterrupt:
#         time.sleep(1)
#         stmInstance.sts3032.stop()
#         pass


if __name__ == "__main__":
    main()
