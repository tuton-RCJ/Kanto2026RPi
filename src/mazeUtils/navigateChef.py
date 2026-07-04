from . import mazeEnums, moveTile
from .device import deviceEnums
from config import get_logger
import time
from collections import defaultdict

# 注文(-2~2)ごとに集める被災者(食材)の deviceEnums.UnitVStatus 値
ORDER_RECIPES: dict[int, tuple[int, ...]] = {
    -2: (1, 4, 5),
    -1: (2, 3, 4),
    0: (3, 4, 5),
    1: (1, 2, 3),
    2: (1, 2, 5),
}

# 注文・受け渡し時に 1st Black タイルで両ロボットが静止する秒数
EXCHANGE_STILL_SECONDS = 5.0
# ゴールで静止する秒数
GOAL_STILL_SECONDS = 5.0

FIRST_BLACK_TILE = (20, 18)

def goToFirstBlackTileFromStart(mapInstance,stmInstance, lidarInstance):
    """
    @brief 最初の黒タイルまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go to the first black tile.")
    assert mapInstance.currentPosition[0] == 20 and mapInstance.currentPosition[1] == 20, f"Current position must be (20, 20) at the start. Now position is {mapInstance.currentPosition}."
    for i in range(2):
        moveTile.moveTile(mazeEnums.absDirection.NORTH, mapInstance, stmInstance, lidarInstance)
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():  # LoP検出時は経路を中断
            return

def goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance, setIngredient: bool = False, findingredient: list[str] = None):
    """
    @brief 2つ目の黒タイルまで移動する 
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    @param setIngredient: 材料を設定するかどうか
    @param findingredient: 設定する材料の名前のリスト, None ならば設定しない
    """
    assert mapInstance.currentPosition[0] == 20 and mapInstance.currentPosition[1] == 18, f"Current position must be (20, 18) before moving to the second black tile. Now position is {mapInstance.currentPosition}."

    logger = get_logger(__name__)
    logger.info("Starting to go to the second black tile.")
    ingredientsCordinates = set()
    if findingredient:
        for ingredient in findingredient:
            coordinates = mapInstance.getIngredientCoordinates(ingredient)
            if coordinates is None:
                logger.warning(f"Ingredient '{ingredient}' has not been mapped yet. Skipping it.")
                continue
            ingredientsCordinates.add(coordinates)
    for i in range(7):
        moveTile.moveTile(mazeEnums.absDirection.EAST, mapInstance, stmInstance, lidarInstance)
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():  # LoP検出時は経路を中断
            return
        victimCountDict = defaultdict(int)
        if setIngredient:
            t = time.time()
            stmInstance.update()
            while time.time() - t < 0.3:
                nowVictim = stmInstance.unitv.getStatus()
                if nowVictim[deviceEnums.Side.LEFT] != deviceEnums.UnitVStatus.NOTHING:
                    victimCountDict[nowVictim[deviceEnums.Side.LEFT]] += 1
                stmInstance.update()
            if len(victimCountDict) > 0:
                mostCommonVictim = max(victimCountDict, key=victimCountDict.get)
                logger.info(f"Detected victim: {mostCommonVictim.name} at position {mapInstance.currentPosition}.")
                if setIngredient:
                    setIngredients(mapInstance, stmInstance, mostCommonVictim.name)
        if findingredient and mapInstance.currentPosition in ingredientsCordinates:
            logger.info(f"Getting ingredient at position {mapInstance.currentPosition}.")
            moveTile.flashLED(mapInstance, stmInstance, 3, 0.5, (255,255,255))
       
def goToFirstBlackTileFromSecond(mapInstance, stmInstance, lidarInstance):
    """
    @brief 2つ目の黒タイルから最初の黒タイルまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go back to the first black tile from the second black tile.")
    assert mapInstance.currentPosition[0] == 27 and mapInstance.currentPosition[1] == 18, f"Current position must be (27, 18) before moving back to the first black tile. Now position is {mapInstance.currentPosition}."
    for i in range(7):
        moveTile.moveTile(mazeEnums.absDirection.WEST, mapInstance, stmInstance, lidarInstance)
        stmInstance.update()
        if stmInstance.switch.getToggleSwitch1():  # LoP検出時は経路を中断
            return

def setIngredients(mapInstance, stmInstance, ingredient: str):
    """
    @brief 材料 str を現在の位置に設定する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param ingredient: 設定する材料の名前
    """
    logger = get_logger(__name__)
    logger.info(f"Setting ingredient '{ingredient}' at current position.")
    mapInstance.setIngredients(ingredient)

def _stayStillOnTile(stmInstance, seconds: float):
    """
    @brief モーターを止めて現在のタイル上で指定秒数静止する
    @param stmInstance: STMのインスタンス
    @param seconds: 静止する秒数
    """
    stmInstance.sts3032.stop()
    startTime = time.time()
    while time.time() - startTime < seconds:
        stmInstance.update()


def orderToIngredients(order: int) -> list[str]:
    """
    @brief 注文(-2~2)を集めるべき食材(UnitVStatusの名前)のリストに変換する
    @param order: ロボットAから受信した注文値
    @return mazeMap.ingredients のキーとして使う UnitVStatus.name のリスト
    """
    return [deviceEnums.UnitVStatus(v).name for v in ORDER_RECIPES[order]]


def waitForOrder(peer, mapInstance, stmInstance) -> int:
    """
    @brief 1st Black タイルで静止したままロボットAからの注文を待ち、受信後さらに5秒間静止する
    @param peer: WirelessPeer のインスタンス
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @return 受信した注文値 (-2~2)
    """
    logger = get_logger(__name__)
    assert mapInstance.currentPosition[0] == FIRST_BLACK_TILE[0] and mapInstance.currentPosition[1] == FIRST_BLACK_TILE[1], f"Current position must be {FIRST_BLACK_TILE} to take an order. Now position is {mapInstance.currentPosition}."
    stmInstance.sts3032.stop()
    logger.info("Waiting for an order from robot A at the first black tile.")
    while True:
        stmInstance.update()
        order = peer.take_order()
        if order is None:
            continue
        if order not in ORDER_RECIPES:
            logger.error(f"Received invalid order: {order}. Keep waiting.")
            continue
        break
    logger.info(f"Received order {order}. Staying still for {EXCHANGE_STILL_SECONDS} seconds.")
    _stayStillOnTile(stmInstance, EXCHANGE_STILL_SECONDS)
    return order


def handOverDish(peer, mapInstance, stmInstance) -> None:
    """
    @brief 1st Black タイルで DISH_READY をロボットAに通知し、ACK後5秒間静止して皿を受け渡す
    @param peer: WirelessPeer のインスタンス
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    """
    logger = get_logger(__name__)
    assert mapInstance.currentPosition[0] == FIRST_BLACK_TILE[0] and mapInstance.currentPosition[1] == FIRST_BLACK_TILE[1], f"Current position must be {FIRST_BLACK_TILE} to hand over a dish. Now position is {mapInstance.currentPosition}."
    stmInstance.sts3032.stop()
    logger.info("Notifying robot A that the dish is ready.")
    while not peer.send_dish_ready():
        stmInstance.update()
    logger.info(f"Robot A acknowledged. Staying still for {EXCHANGE_STILL_SECONDS} seconds to hand over the dish.")
    _stayStillOnTile(stmInstance, EXCHANGE_STILL_SECONDS)


def goToGoal(mapInstance, stmInstance, lidarInstance):
    """
    @brief ゴールまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go to the goal.")
    if mapInstance.currentPosition[0] == 20 and mapInstance.currentPosition[1] == 20:
        goToFirstBlackTileFromStart(mapInstance, stmInstance, lidarInstance)
    if mapInstance.currentPosition[0] == 20 and mapInstance.currentPosition[1] == 18:
        goToSecondBlackTileFromFirst(mapInstance, stmInstance, lidarInstance)
    if mapInstance.currentPosition[0] == 27 and mapInstance.currentPosition[1] == 18:
        for i in range(2):
            moveTile.moveTile(mazeEnums.absDirection.SOUTH, mapInstance, stmInstance, lidarInstance)
    logger.info(f"Reached the goal position. Staying still for {GOAL_STILL_SECONDS} seconds.")
    _stayStillOnTile(stmInstance, GOAL_STILL_SECONDS)
