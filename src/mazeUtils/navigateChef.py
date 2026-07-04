from . import mazeEnums, moveTile
from .device import deviceEnums
from config import get_logger
import time
from collections import defaultdict

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
            ingredientsCordinates.add(mapInstance.getIngredientCoordinates(ingredient))
    for i in range(7):
        moveTile.moveTile(mazeEnums.absDirection.EAST, mapInstance, stmInstance, lidarInstance)
        stmInstance.update()
        if stmInstance.getToggleSwitch1():
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
    logger.info("Reached the goal position.")
