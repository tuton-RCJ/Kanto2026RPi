from config import setup_logging, get_logger
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

# ロギング設定
setup_logging()
logger = get_logger(__name__)


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
    moveTile.turnOffLED(stmInstance)

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    mapInstance = mazeMap.mazeMap()
    stmInstance.rearSTM.camled((255, 255, 255))
    lidarInstance = LiDAR.initializeLidar()
    stmInstance.rearSTM.playMusic(buzzerSongs.start)
    stmInstance.rearSTM.send_message("Get Ready!")
    stmInstance.update()

    while stmInstance.switch.getToggleSwitch1():
        if stmInstance.switch.getPushSwitch1():
            stmInstance.rearSTM.playMusic(buzzerSongs.start)
            time.sleep(1)
        if not stmInstance.update():
            logger.warning("STM update failed. Retrying...")
            stmInstance.switch.setValue(pushSwitch1=False, toggleSwitch1=True)
            time.sleep(0.1)
            continue
            
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    gameStartTime = time.time()
    time.sleep(1)
    mapBroken = False
    try:
        while True:
            stmInstance.update()
            moveTile.detectWall(lidarInstance, mapInstance, stmInstance, enableOverwrite=True)
            logger.info("Initial Map:")
            logger.info(mapInstance.renderKnownTileAndWall())
            if not mapBroken:
                mapInstance.saveCache()
            mapBroken = False

            nextDirection = mapInstance.getNearestUnexploredTile()
            logger.info(f"Next Direction: {nextDirection}")
            logger.info("Exploration started.")
            isLoP = False
            while nextDirection is not None and not isLoP:

                if (
                    mazeConstraints.RETURN_JUDGE_MODE
                    == mazeEnums.returnJudgeMode.ONLY_TIME_BASED
                ):
                    if time.time() - gameStartTime > mazeConstraints.RETURN_TIME_THRESHOLD_SEC:
                        logger.info("Time's up! Starting return to the starting point.")
                        break
                if (mazeConstraints.RETURN_JUDGE_MODE
                    == mazeEnums.returnJudgeMode.TIME_BASED_WITH_DISTANCE):
                    _costToStart = mapInstance.getCostToStartTile()
                    if _costToStart >= 0:
                        estReturnTime = time.time() + _costToStart * 1.5
                        if estReturnTime - gameStartTime > mazeConstraints.RETURN_TIME_WITH_DISTANCE_THRESHOLD_SEC:
                            logger.info("Estimated return time exceeds threshold! Starting return to the starting point.")
                            break

                # nextDirection = mapInstance.getNearestUnexploredTile()

                if nextDirection is None:
                    continue
                for direction in nextDirection:
                    before_movement_pos = mapInstance.currentPosition
                    isBlack, stopped, resetMapData = moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )
                    if resetMapData:
                        logger.warning("Map data reset due to wall detection error.")
                        stmInstance.rearSTM.playMusic(buzzerSongs.mappingError)
                        break
                    logger.info(mapInstance.renderKnownTileAndWall())

                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        logger.info("Exploration resumed.")

                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLoP = True
                        break
                    if before_movement_pos == mapInstance.currentPosition:
                        logger.warning("Position did not change after movement. Possible error in movement or wall detection.")
                        break
                    # moveTile.turnOffLED(stmInstance)
                    
                # 周囲にU字型の未探索タイルがある場合、優先的にU字型のタイルに進む
                detectedUshapedTile = False
                if mazeConstraints.PRIORITIZE_UNEXPLORED_U_SHAPED_TILE and mapInstance.getTileType() != mazeEnums.tileType.RED:
                    for direction in mazeEnums.absDirection:
                        next_tile = mapInstance.getTileType(direction)
                        if next_tile is None:
                            continue
                        if next_tile != mazeEnums.tileType.UNKNOWN:
                            continue
                        # pts = LiDAR.getLiDARScan(lidarInstance)
                        isUshaped = LiDAR.detectUshapedTile_ROI(direction.value-mapInstance.frontDirection.value)
                        if isUshaped:
                            logger.info(f"U-shaped unexplored tile detected in direction {direction}. Prioritizing this tile.")
                            nextDirection = [direction]
                            detectedUshapedTile = True
                            break
                if not detectedUshapedTile:
                    nextDirection = mapInstance.getNearestUnexploredTile()
            if isLoP:
                isLoP = False
                continue

            ##### 帰還開始 #####
            
            stmInstance.rearSTM.send_message(f"Returning to Start. Time: {int(time.time() - gameStartTime)}s")

            stmInstance.rearSTM.playMusic(buzzerSongs.hotaru)
            start_tile_position =  mapInstance.estimateStartTile()
            if start_tile_position is None:
                logger.error("Start tile position could not be estimated. Cannot return to start.")
                mapInstance.resetMapData()
                mapBroken = True
                isLop = True
                continue
            returnPath = mapInstance.getPathTo(start_tile_position)
            logger.info(f"Return Path: {returnPath}")
            
            if returnPath is None:
                stmInstance.rearSTM.playMusic(buzzerSongs.mappingError)
                logger.error("Return path could not be calculated. Cannot return to start.")
                mapInstance.resetMapData()
                mapBroken = True
                isLop = True
                continue

            isLop = False
            if returnPath:
                for direction in returnPath:
                    isBlack, stopped, resetMapData = moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )

                    if resetMapData:
                        logger.warning("Map data reset due to wall detection error.")
                        stmInstance.rearSTM.playMusic(buzzerSongs.mappingError)
                        mapBroken = True
                        isLop = True
                        break

                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        logger.info("Exploration resumed.")
                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLop = True
                        break

                    logger.info(mapInstance.renderKnownTileAndWall())
            if isLop:
                continue

            logger.info("Robot now at the starting position, Congratulations!")
            stmInstance.rearSTM.playMusic(buzzerSongs.matuken)
            moveTile.flashLED(
                stmInstance,
                mapInstance,
                loopCount=5,
                intervalSec=1,
                color=(255, 255, 255),
            )  # Flash white LED to indicate completion
            stmInstance.sts3032.stop()
            logger.info(mapInstance.renderKnownTileAndWall())

            ### LoP検出後の再開処理
            while not stmInstance.switch.getToggleSwitch1():
                stmInstance.update()
            logger.warning("detect LoP. back to last check point.")

            if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                logger.info("Exploration resumed.")
                _recover_from_lop(stmInstance, mapInstance, lidarInstance)
            continue

    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
        raise
    except Exception:
        logger.exception("An unexpected error occurred:")
    finally:
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()
        moveTile.turnOffLED(stmInstance)


if __name__ == "__main__":
    main()
