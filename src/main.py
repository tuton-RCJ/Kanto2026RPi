from config import setup_logging, get_logger
from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import uart_comunication
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
    mapInstance.currentPosition = (20, 20, 0)  # LoP後は必ずスタートタイルに置き直される
    time.sleep(1)
    stmInstance.update()
    moveTile.turnOffLED(stmInstance)


def _handle_lop_if_any(stmInstance, mapInstance, lidarInstance) -> bool:
    """
    @brief LoPが発生していれば復帰を待ってリカバリし、Trueを返す。発生していなければFalseを返す。
    """
    if _detect_and_wait_for_lop(stmInstance):
        logger.warning("detect LoP. back to last check point.")
        logger.info("Exploration resumed.")
        _recover_from_lop(stmInstance, mapInstance, lidarInstance)
        return True
    return False

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    mapInstance = mazeMap.mazeMap()
    peer = uart_comunication.WirelessPeer()
    peer.start()
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
            if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                continue
            navigateChef.goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance, setIngredient=True)
            if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                continue
            navigateChef.goToFirstBlackTileFromSecond(mapInstance, stmInstance, lidarInstance)
            if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                continue
            break

        pendingOrder = None  # LoPを跨いでも作りかけの注文を保持する
        while True:
            while mapInstance.sendedDishes < 3:
                if time.time() - gameStartTime > 420:
                    logger.info("Game time exceeded. Exiting.")
                    break
                stmInstance.update()
                moveTile.detectWall(lidarInstance, mapInstance, stmInstance, enableOverwrite=True)
                logger.info("Initial Map:")
                logger.info(mapInstance.renderKnownTileAndWall())
                if not mapBroken:
                    mapInstance.saveCache()
                mapBroken = False
                if mapInstance.currentPosition[0] == 20 and mapInstance.currentPosition[1] == 20:
                    navigateChef.goToFirstBlackTileFromStart(mapInstance, stmInstance, lidarInstance)
                    if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                        continue
                if pendingOrder is None:
                    pendingOrder = navigateChef.waitForOrder(peer, mapInstance, stmInstance)
                navigateChef.goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance, setIngredient=False, findingredient=navigateChef.orderToIngredients(pendingOrder))
                if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                    continue 
                navigateChef.goToFirstBlackTileFromSecond(mapInstance, stmInstance, lidarInstance)
                if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                    continue  
                navigateChef.handOverDish(peer, mapInstance, stmInstance)
                mapInstance.sendedDishes += 1
                pendingOrder = None

            navigateChef.goToGoal(mapInstance, stmInstance, lidarInstance)
            if _handle_lop_if_any(stmInstance, mapInstance, lidarInstance):
                continue
            break

    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
        raise
    except Exception:
        logger.exception("An unexpected error occurred:")
    finally:
        peer.stop()
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()
        moveTile.turnOffLED(stmInstance)


if __name__ == "__main__":
    main()
