from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
import ydlidar
import time


def _detect_and_wait_for_lop(stmInstance) -> bool:
    """
    @brief LoPを検出して、再開するまで待機する
    @param stmInstance: STMのインスタンス
    @return LoPが発生した場合は再開するまで待機してからTrueを返す。そうでない場合はFalseを返す。
    """
    toggleswitchFlag = False
    stmInstance.update()
    while stmInstance.switch.getToggleSwitch1():
        # 進行停止中の待機
        stmInstance.update()
        toggleswitchFlag = True
    return toggleswitchFlag


def _recover_from_lop(stmInstance, mapInstance, lidarInstance):
    nowAngle = stmInstance.gyro.getValue().heading
    nowDirection = mazeEnums.absDirection.NORTH
    error = 1e9
    for direction in mazeEnums.absDirection:
        diff = abs(nowAngle - direction.value)
        if diff > 180:
            diff = 360 - diff
        if diff < error:
            error = diff
            nowDirection = direction

    mapInstance.loadCache(nowDirection=nowDirection)
    time.sleep(1)
    stmInstance.update()
    moveTile.turnOffLED(stmInstance, mapInstance)


def main():
    stmInstance = stm.STM()
    stmInstance.update()
    mapInstance = mazeMap.mazeMap()

    mapInstance.arduinoNanoEvery.camled((0, 0, 0))
    lidarInstance = LiDAR.initializeLidar()
    stmInstance.buzzer.playMusic(buzzerSongs.start)
    stmInstance.update()
    while stmInstance.switch.getToggleSwitch1():
        stmInstance.update()
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    time.sleep(1)
    try:
        while True:
            stmInstance.update()
            moveTile.detectWall(lidarInstance, mapInstance, stmInstance)
            mapInstance.saveCache()
            print("Initial Map:")
            print(mapInstance.renderKnownTileAndWall())

            nextDirection = mapInstance.getNearestUnexploredTile()
            print(f"Next Direction: {nextDirection}")
            print("Exploration started.")
            isLoP = False
            while nextDirection is not None and not isLoP:
                nextDirection = mapInstance.getNearestUnexploredTile()
                if nextDirection is None:
                    continue
                for direction in nextDirection:
                    isBlack, stopped = moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )
                    print(mapInstance.renderKnownTileAndWall())


                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        print("Exploration resumed.")

                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLoP = True
                        break
                    moveTile.turnOffLED(stmInstance, mapInstance)
                nextDirection = mapInstance.getNearestUnexploredTile()
            if isLoP:
                isLoP = False
                continue
            
            ##### 帰還開始 #####
            
            stmInstance.buzzer.playMusic(buzzerSongs.hotaru)
            returnPath = mapInstance.getPathTo((20, 20))
            print(f"Return Path: {returnPath}")
            
            isLop = False   
            if returnPath:
                for direction in returnPath:
                    moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )

                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        print("Exploration resumed.")
                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLop = True
                        break
                    
                    print(mapInstance.renderKnownTileAndWall())
            if isLop:
                continue
            
            
            print("Robot now at the starting position, Congratulations!")
            stmInstance.buzzer.playMusic(buzzerSongs.matuken)
            moveTile.flashLED(
                stmInstance,
                mapInstance,
                loopCount=5,
                intervalSec=1,
                color=[255, 255, 255],
            )  # Flash white LED to indicate completion
            stmInstance.sts3032.stop()
            print(mapInstance.renderKnownTileAndWall())
            
            ### LoP検出後の再開処理
            while not stmInstance.switch.getToggleSwitch1():
                stmInstance.update()
            print("detect LoP. back to last check point.")
            
            if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                print("Exploration resumed.")
                _recover_from_lop(stmInstance, mapInstance, lidarInstance)
            continue

    except KeyboardInterrupt:
        print("Program interrupted by user.")
        raise
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()
        moveTile.turnOffLED(stmInstance, mapInstance)

if __name__ == "__main__":
    main()
