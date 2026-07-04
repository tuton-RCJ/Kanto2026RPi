from . import mazeEnums, moveTile
from .device import deviceEnums
from config import get_logger
import time
from collections import defaultdict

def goToFirstBlackTileFromStart(stmInstance, mapInstance, lidarInstance):
    """
    @brief 最初の黒タイルまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go to the first black tile.")
    assert mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 0, "Current position must be (0, 0) at the start."
    for i in range(2):
        moveTile.movetile(mazeEnums.absDirection.NORTH, stmInstance, mapInstance, lidarInstance)

def goToSecondBlackTileFromFirst(stmInstance, mapInstance, lidarInstance, setIngredient: bool = False, findingredient: list[str] = None):
    """
    @brief 2つ目の黒タイルまで移動する 
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    @param setIngredient: 材料を設定するかどうか
    @param findingredient: 設定する材料の名前のリスト, None ならば設定しない
    """
    assert mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 2, "Current position must be (0, 2) before moving to the second black tile."

    logger = get_logger(__name__)
    logger.info("Starting to go to the second black tile.")
    ingredientsCordinates = set()
    if findingredient:
        for ingredient in findingredient:
            ingredientsCordinates.add(mapInstance.getIngredientCoordinates(ingredient))
    for i in range(7):
        moveTile.movetile(mazeEnums.absDirection.EAST, stmInstance, mapInstance, lidarInstance)
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
                logger.info(f"Detected victim: {mostCommonVictim.name} at position {mapInstance.CurrentPosition}.")
                if setIngredient:
                    setIngredients(stmInstance, mapInstance, mostCommonVictim.name)
        if findingredient and mapInstance.CurrentPosition in ingredientsCordinates:
            logger.info(f"Getting ingredient at position {mapInstance.CurrentPosition}.")
            moveTile.flashLED(stmInstance, mapInstance, 3, 0.5, (255,255,255))
       
def goToFirstBlackTileFromSecond(stmInstance, mapInstance, lidarInstance):
    """
    @brief 2つ目の黒タイルから最初の黒タイルまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go back to the first black tile from the second black tile.")
    assert mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 9, "Current position must be (0, 9) before moving back to the first black tile."
    for i in range(7):
        moveTile.movetile(mazeEnums.absDirection.WEST, stmInstance, mapInstance, lidarInstance)

def setIngredients(stmInstance, mapInstance, ingredient: str):
    """
    @brief 材料 str を現在の位置に設定する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param ingredient: 設定する材料の名前
    """
    logger = get_logger(__name__)
    logger.info(f"Setting ingredient '{ingredient}' at current position.")
    mapInstance.setIngredients(ingredient)

def goToGoal(stmInstance, mapInstance, lidarInstance):
    """
    @brief ゴールまで移動する
    @param stmInstance: STMのインスタンス
    @param mapInstance: mazeMapのインスタンス
    @param lidarInstance: LiDARのインスタンス
    """
    logger = get_logger(__name__)
    logger.info("Starting to go to the goal.")
    if mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 0:
        goToFirstBlackTileFromStart(stmInstance, mapInstance, lidarInstance)
    if mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 2:
        goToSecondBlackTileFromFirst(stmInstance, mapInstance, lidarInstance)
    if mapInstance.CurrentPosition[0] == 0 and mapInstance.CurrentPosition[1] == 9:
        for i in range(2):
            moveTile.movetile(mazeEnums.absDirection.SOUTH, stmInstance, mapInstance, lidarInstance)
    logger.info("Reached the goal position.")