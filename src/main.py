from config import setup_logging, get_logger
from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
from mazeUtils import navigateChef
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
        stmInstance.update()
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    gameStartTime = time.time()
    time.sleep(1)
    mapBroken = False
    try:
        while True:
            navigateChef.goToFirstBlackTileFromStart(mapInstance, stmInstance, lidarInstance)
            navigateChef.goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance, setIngredient=True)
            navigateChef.goToFirstBlackTileFromSecond(mapInstance, stmInstance, lidarInstance)
            if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                logger.warning("detect LoP. back to last check point.")
                logger.info("Exploration resumed.")
                _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                continue
            break

        while True:
            while mapInstance.sendedDishes < 3:
                if time.time() - gameStartTime > 60:
                    logger.info("Game time exceeded. Exiting.")
                    break
                stmInstance.update()
                moveTile.detectWall(lidarInstance, mapInstance, stmInstance, enableOverwrite=True)
                logger.info("Initial Map:")
                logger.info(mapInstance.renderKnownTileAndWall())
                if not mapBroken:
                    mapInstance.saveCache()
                mapBroken = False
                # TODO: wait for order from robot A
                navigateChef.goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance, setIngredient=False, findingredient=["tomato", "onion", "cheese"])
                navigateChef.goToFirstBlackTileFromSecond(mapInstance, stmInstance, lidarInstance)
                # TODO: wait for sending dish to robot A
                mapInstance.sendedDishes += 1 if stmInstance.switch.getToggleSwitch1() else 0
                if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                    logger.warning("detect LoP. back to last check point.")
                    logger.info("Exploration resumed.")
                    _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                    continue

            navigateChef.goToGoal(mapInstance, stmInstance, lidarInstance)
            if _detect_and_wait_for_lop(stmInstance):  # LoP検出後の再開処理
                logger.warning("detect LoP. back to last check point.")
                logger.info("Exploration resumed.")
                _recover_from_lop(stmInstance, mapInstance, lidarInstance)
                continue
            break

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
