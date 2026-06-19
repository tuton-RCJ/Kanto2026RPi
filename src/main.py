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

GAME_TIME_SEC = 80 # 競技走行時間

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    mapInstance = mazeMap.mazeMap()
    stmInstance.rearSTM.camled((255, 255, 255))
    lidarInstance = LiDAR.initializeLidar()
    stmInstance.buzzer.playMusic(buzzerSongs.start)
    stmInstance.update()
    while stmInstance.switch.getToggleSwitch1():
        stmInstance.update()
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    gameStartTime = time.time()
    time.sleep(1)
    try:
        while True:
            stmInstance.update()
            moveTile.detectWall(lidarInstance, mapInstance, stmInstance)
            logger.info("Initial Map:")
            logger.info(mapInstance.renderKnownTileAndWall())

            nextDirection = mapInstance.getNearestUnexploredTile()
            logger.info(f"Next Direction: {nextDirection}")
            logger.info("Exploration started.")
            isLoP = False
            while nextDirection is not None and not isLoP:
                if time.time() - gameStartTime > GAME_TIME_SEC:
                    logger.info("Time's up! Starting return to the starting point.")
                    break
                # nextDirection = mapInstance.getNearestUnexploredTile()
                if nextDirection is None:
                    continue
                for direction in nextDirection:
                    isBlack, stopped = moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )
                    logger.info(mapInstance.renderKnownTileAndWall())

                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        logger.info("Exploration resumed.")

                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLoP = True
                        break
                    # moveTile.turnOffLED(stmInstance)
                nextDirection = mapInstance.getNearestUnexploredTile()
            if isLoP:
                isLoP = False
                continue

            ##### 帰還開始 #####

            stmInstance.buzzer.playMusic(buzzerSongs.hotaru)
            returnPath = mapInstance.getPathTo((20, 20, 0))
            logger.info(f"Return Path: {returnPath}")

            isLop = False
            if returnPath:
                for direction in returnPath:
                    moveTile.moveNextTile(
                        direction, mapInstance, stmInstance, lidarInstance
                    )

                    if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                        logger.info("Exploration resumed.")
                        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                        isLop = True
                        break

                    logger.info(mapInstance.renderKnownTileAndWall())
            if isLop:
                continue

            logger.info("Robot now at the starting position, Congratulations!")
            stmInstance.buzzer.playMusic(buzzerSongs.matuken)
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
