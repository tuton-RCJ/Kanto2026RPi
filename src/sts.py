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
from mazeUtils import mazeEnums
import math
from mazeUtils import mazeConstraints
import ydlidar
import time
import math

MOVE_1cm_TIME = 0.050


def main():
    stmInstance = stm.STM()
    nowDist = 0
    lastTime = time.time()
    nowhight = 0
    stmInstance.update()
    print(
        stmInstance.gyro.getValue().heading,
        stmInstance.gyro.getValue().roll,
        stmInstance.gyro.getValue().pitch,
    )
    time.sleep(3)
    lastToFDist = stmInstance.tof.getDistance()[2]
    try:
        while stmInstance.gyro.getValue().roll < 5:

            stmInstance.update()
            movedDist = (time.time() - lastTime) / MOVE_1cm_TIME
            # movedDist = stmInstance.tof.getDistance()[2] - lastToFDist
            # lastToFDist = stmInstance.tof.getDistance()[2]
            lastTime = time.time()
            nowDist += movedDist * math.cos(
                math.radians(stmInstance.gyro.getValue().roll)
            )
            nowhight += movedDist * math.sin(
                math.radians(stmInstance.gyro.getValue().roll)
            )
            print(nowDist)

            # if nowDist >= 30:
            #     stmInstance.sts3032.stop()
            #     nowDist = 0
            #     print("hight", nowhight, "cm")
            #     time.sleep(1)
            #     lastTime = time.time()

            stmInstance.sts3032.setMotorSpeed(
                {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50}
            )
            time.sleep(0.01)
            # while True:
            #     s = input()
            #     stmInstance.update()
            #     print(
            #         stmInstance.gyro.getValue().heading,
            #         stmInstance.gyro.getValue().roll,
            #         stmInstance.gyro.getValue().pitch,
            #     )
        stmInstance.sts3032.stop()
        time.sleep(2)
        
        nowDist = 0
        nowhight = 0
        lastTime=time.time()
        while stmInstance.gyro.getValue().roll > 3:

            stmInstance.update()
            movedDist = (time.time() - lastTime) / MOVE_1cm_TIME
            # movedDist = stmInstance.tof.getDistance()[2] - lastToFDist
            # lastToFDist = stmInstance.tof.getDistance()[2]
            lastTime = time.time()
            nowDist += movedDist * math.cos(
                math.radians(stmInstance.gyro.getValue().roll)
            )
            nowhight += movedDist * math.sin(
                math.radians(stmInstance.gyro.getValue().roll)
            )
            print(nowDist)

            # if nowDist >= 30:
            #     stmInstance.sts3032.stop()
            #     nowDist = 0
            #     print("hight", nowhight, "cm")
            #     time.sleep(1)
            #     lastTime = time.time()

            stmInstance.sts3032.setMotorSpeed(
                {deviceEnums.Side.LEFT: 50, deviceEnums.Side.RIGHT: 50}
            )
            time.sleep(0.01)
        stmInstance.sts3032.stop()
        print("hight", nowhight, "cm")
        print("dist", nowDist, "cm")
    except KeyboardInterrupt:
        time.sleep(1)
        stmInstance.sts3032.stop()
        pass


if __name__ == "__main__":
    main()
