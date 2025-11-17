from tkinter import IntVar
import json
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

from collections import defaultdict
import os
import heapq
import math
from queue import PriorityQueue
import time


@dataclass
class MappingDataTileInfo:
    fieldCoord: tuple[int, int] = (0, 0)  # (x,y) フィールド座標(タイルと壁を含む)
    tileCoord: tuple[int, int] = (0, 0)  # (x,y) タイル座標(タイルのみ)
    tileType: int = 0  # 0: normal, 1: black, 2: swamp, 3: stair
    visitTileCount: int = 0  # そのタイルに訪れた回数
    visitWallCount: dict[int, int] = field(
        default_factory=lambda: {0: 0, 90: 0, 180: 0, 270: 0}
    )  # そのタイルを囲む壁を見た回数 絶対方位
    wallStatus: dict[int, int] = field(
        default_factory=lambda: {0: 0, 90: 0, 180: 0, 270: 0}
    )  # そのタイルを囲む壁の状態 0:未発見 1:発見 絶対方位


@dataclass
class MappingDataWallInfo:
    position: tuple[int] = (0, 0)  # (x,y) #フィールド座標(タイルと壁を含む), 辞書のKeyと一致
    isWall: bool = False  # 壁かどうか # True: 壁, False: 壁なし


# マッピング用フィールド記憶データクラス
class MappingField:
    def __init__(self):
        self.name = ""
        self.mapData: dict[tuple[int],
                           MappingDataTileInfo | MappingDataWallInfo] = {}
        self.startTile = (0, 0)
    # タイル座標とフィールド座標の変換

    def tileCoord2FieldCoord(self, tile_x, tile_y):
        return (tile_x * 2 + self.startTile[0], tile_y * 2 + self.startTile[1])
    # フィールド座標とタイル座標の変換

    def fieldCoord2TileCoord(self, field_x, field_y):
        return ((field_x - self.startTile[0]) // 2, (field_y - self.startTile[1]) // 2)

    # タイル情報の登録・更新。タイル座標と、タイルタイプ、訪問回数の増分、壁の訪問回数の増分、壁の状態を示す辞書を与える

    def registerTile(self, tileCoord: tuple[int], tileType: int = -1, incrementVisitTileCount: int = 0, incrementVisitWallCount: dict[int, int] = {0: 0, 90: 0, 180: 0, 270: 0}, wallStatus: dict[int, int] = {0: -1, 90: -1, 180: -1, 270: -1}):
        tileInfo = MappingDataTileInfo()
        fieldCoord = self.tileCoord2FieldCoord(*tileCoord)
        if fieldCoord in self.mapData.keys():
            tileInfo = self.mapData[fieldCoord]
            if tileType != -1:
                tileInfo.tileType = tileType
            for dir in [0, 90, 180, 270]:
                if wallStatus.get(dir, -1) != -1:
                    tileInfo.wallStatus[dir] = wallStatus[dir]
        else:
            tileInfo.fieldCoord = fieldCoord
            tileInfo.tileCoord = tileCoord
            tileInfo.tileType = tileType if tileType != -1 else 0
            for dir in [0, 90, 180, 270]:
                tileInfo.wallStatus[dir] = wallStatus.get(dir, -1)
        tileInfo.visitTileCount += incrementVisitTileCount
        for dir in [0, 90, 180, 270]:
            tileInfo.visitWallCount[dir] += incrementVisitWallCount.get(dir, 0)
        self.mapData[fieldCoord] = tileInfo

    # 壁情報の登録。タイル座標と、そのタイルに隣接する四方向の壁の有無を示す辞書を与える
    def registerWall(self, tileCoord: tuple[int], isWallDict: dict[int, bool]):
        fieldCoord = self.tileCoord2FieldCoord(*tileCoord)
        for dir, isWall in isWallDict.items():
            wallFieldCoord = None
            if dir == 0:
                wallFieldCoord = (fieldCoord[0] + 1, fieldCoord[1])
            elif dir == 90:
                wallFieldCoord = (fieldCoord[0], fieldCoord[1] - 1)
            elif dir == 180:
                wallFieldCoord = (fieldCoord[0] - 1, fieldCoord[1])
            elif dir == 270:
                wallFieldCoord = (fieldCoord[0], fieldCoord[1] + 1)
            if wallFieldCoord is not None:
                wallInfo = MappingDataWallInfo()
                wallInfo.position = wallFieldCoord
                wallInfo.isWall = isWall
                self.mapData[wallFieldCoord] = wallInfo
    # 指定したタイルから隣接四方向の壁の有無を返す

    def getWallInfo(self, tileCoord: tuple[int]) -> dict[int, bool]:
        fieldCoord = self.tileCoord2FieldCoord(*tileCoord)
        wallInfo = {}
        directions = {
            0: (1, 0),
            90: (0, -1),
            180: (-1, 0),
            270: (0, 1)
        }
        for direction, (dx, dy) in directions.items():
            neighbor = (fieldCoord[0] + dx, fieldCoord[1] + dy)
            if neighbor in self.mapData.keys() and isinstance(self.mapData[neighbor], MappingDataWallInfo):
                wallInfo[direction] = self.mapData[neighbor].isWall
            else:
                wallInfo[direction] = None  # 壁情報が未登録
        return wallInfo

    # タイル情報の取得
    def getTileInfo(self, tileCoord: tuple[int]) -> Optional[MappingDataTileInfo]:
        fieldCoord = self.tileCoord2FieldCoord(*tileCoord)
        if fieldCoord in self.mapData.keys() and isinstance(self.mapData[fieldCoord], MappingDataTileInfo):
            return self.mapData[fieldCoord]
        else:
            return None  # タイル情報が未登録


@dataclass
class dijkstraResult:
    cost: int
    route: List[tuple[int]]

# マッピングクラス。 マッピング用フィールド記憶データを持ち、経路探索などを行う


goStraightCost = 3
turn90Cost = 1


class Mapping:
    def __init__(self):
        self.mappingField: MappingField = MappingField()

    # 与えられたタイルから隣接四方向タイルへのコストを計算して返す
    # 黒タイル、沼、階段などは後日コスト変化の実装を追加する
    def calcNextTileCost(self, current: tuple[int]) -> dict[int, int]:
        # 方向は絶対方位
        directions = {
            0: (1, 0),
            90: (0, 1),
            180: (-1, 0),
            270: (0, -1)
        }
        costs = {}
        # 壁の有無を読み込み
        wallInfo = self.mappingField.getWallInfo(current)
        for direction, (dx, dy) in directions.items():
            if wallInfo[direction] == True or wallInfo[direction] is None:
                costs[direction] = math.inf
                continue
            neighbor = (current[0] + dx, current[1] + dy)
            neighborTileInfo = self.mappingField.getTileInfo(neighbor)
            # 隣接タイルが存在しない or 壁がある場合はコスト無限大
            if neighborTileInfo is not None:
                if neighborTileInfo.tileType == 4:  # 沼
                    costs[direction] = 6
                else:
                    costs[direction] = 1
            else:
                costs[direction] = math.inf
        return costs

    # ダイクストラ法による最短経路探索
    # start: 探索開始タイルのフィールド座標 (x,y)
    # type: 探索タイプ "all": 全ての未到達タイル, "nearestUnreached": 最も近い未到達タイル、"unreached": 未到達タイル
    def dijkstra(self, start: tuple[int], startDir: int, searchType: str = "all") -> dict[dijkstraResult]:
        q = PriorityQueue()
        q.put((0, start, startDir))  # (cost, position, direction)
        distances: defaultdict[tuple[tuple[int, int],
                                     int], int] = defaultdict(lambda: math.inf)
        distances[(start, startDir)] = 0
        routes: dict[tuple[tuple[int], int], List[tuple[int]]] = {}
        routes[(start, startDir)] = []  # 経路復元用
        unreached: dict[tuple[int], int] = {}  # 未到達タイルのコスト記録用

        while q.qsize() > 0:
            current_distance, current_position, current_direction = q.get()
            # print(q.qsize(), current_distance, current_position)
            if self.mappingField.getTileInfo(current_position).visitTileCount == 0 and current_position != start:
                # 未到達タイルに到達したらコストを記録
                unreached[current_position] = current_distance
                if searchType == "nearestUnreached":
                    break

            # キュー追加後に距離が更新されていた場合はスキップ
            if current_distance > distances[current_position]:
                continue
            # 隣接タイルへのコストを算出
            next_costs = self.calcNextTileCost(current_position)
            for direction, cost in next_costs.items():
                if math.isinf(cost):
                    continue
                neighbor = (current_position[0] + (1 if direction == 0 else -1 if direction == 180 else 0),
                            current_position[1] + (1 if direction == 90 else -1 if direction == 270 else 0))

                distance = current_distance + cost + goStraightCost + ((turn90Cost*2) if abs(
                    direction-current_direction) == 180 else (turn90Cost if direction != current_direction else 0))

                if distance < distances[(neighbor, direction)]:
                    distances[(neighbor, direction)] = distance
                    q.put((distance, neighbor, direction))
                    routes[(neighbor, direction)] = routes[(current_position, current_direction)] + \
                        [current_position]

        # 復元経路とコストを合わせてreturn
        returnDict = {}
        for (position, direction), cost in distances.items():
            if cost == math.inf:
                continue
            returnDict[position] = min(dijkstraResult(
                cost=cost, route=routes[(position, direction)] + [position]), returnDict.get(position, dijkstraResult(math.inf, [])), key=lambda x: x.cost)

        if searchType == "all":
            return returnDict
        elif searchType == "nearestUnreached" or searchType == "unreached":
            # Unreachedのコストと経路を返す
            returnDict = {k: v for k, v in returnDict.items()
                          if k in unreached.keys()}
            return returnDict
        else:
            raise ValueError("Invalid searchType")

    def __str__(self):
        return f"Mapping(mappingField={self.mappingField})"

    def __repr__(self):
        return self.__str__()

# 探索機クラス。マッピングクラスを持ちながら、ロボットの位置情報等を持ち、実際に探索を指示するクラス。


class Explorer:

    def __init__(self, moveForwardFunc: callable = None, turnFunc: callable = None, drawWallCount: callable = None, drawTileCount: callable = None, getTileInfoFunc: callable = None):

        self.position = (0, 0)
        self.direction = 0  # 初期方向は北（上）
        self.moveForwardFunc = moveForwardFunc
        self.turnFunc = turnFunc
        self.WallCount = defaultdict(int)
        self.drawWallCount = drawWallCount
        self.tileCount = defaultdict(int)
        self.drawTileCount = drawTileCount
        self.notVisitedTiles = set()
        self.getTileInfoFunc = getTileInfoFunc

        self.mapping = Mapping()
        self.runCost = 0

    def move_forward(self):
        # self.updateTileCount()
        # self.updateWallCount()
        self.runCost += goStraightCost
        if self.moveForwardFunc:
            self.moveForwardFunc()
        
        if self.direction == 90:
            self.position = (self.position[0], self.position[1] +1)
        elif self.direction == 270:
            self.position = (self.position[0], self.position[1] - 1)
        elif self.direction == 180:
            self.position = (self.position[0] - 1, self.position[1])
        elif self.direction == 0:
            self.position = (self.position[0] + 1, self.position[1])
        else:
            print("Error: Invalid direction")

    def rotate(self, angle):  # 反時計回りが正
        # self.updateWallCount()
        self.runCost += turn90Cost * (abs(angle) // 90)
        self.direction = (self.direction + angle) % 360
        if self.turnFunc:
            self.turnFunc(angle)

    # def updateWallCount(self):
    #     info = self.field.get_tile_info(
    #         *self.position)
    #     for dir in [(90+self.direction) % 360, (270+self.direction) % 360]:
    #         if info[dir] == "wall":
    #             self.WallCount[(self.position[0], self.position[1], dir)] += 1
    #             if self.drawWallCount:
    #                 self.drawWallCount(
    #                     self.position[0], self.position[1], dir, self.WallCount[(self.position[0], self.position[1], dir)])

    # def updateTileCount(self):
    #     self.mapping.mappingField.registerTile(
    #         self.position, incrementVisitTileCount=1)
    #     if self.drawTileCount:
    #         self.drawTileCount(self.position, self.mapping.mappingField.getTileInfo(
    #             self.position).visitTileCount)

    def dir2NextPos(self, dir):
        direction = (self.direction + dir) % 360
        if direction == 90:
            return (self.position[0], self.position[1]+1)
        elif direction == 270:
            return (self.position[0], self.position[1]-1)
        elif direction == 180:
            return (self.position[0]-1, self.position[1])
        elif direction == 0:
            return (self.position[0]+1, self.position[1])
        else:
            print("Error: Invalid direction")
            return self.position

    def ExploreStep(self):
        info = self.getTileInfoFunc()
        directions = {
            0: (1, 0),
            90: (0, 1),
            180: (-1, 0),
            270: (0, -1)
        }
        notVisitedNeighbors = []
        neighborWalls = {}
        for dir, (dx, dy) in directions.items():  # 絶対方位
            if info[dir] == "wall":
                neighborWalls[dir] = True
            else:
                neighborWalls[dir] = False
                # 未到達か確認
                if self.mapping.mappingField.getTileInfo((self.position[0] + dx, self.position[1] + dy)) is None or self.mapping.mappingField.getTileInfo((self.position[0] + dx, self.position[1] + dy)).visitTileCount == 0:
                    notVisitedNeighbors.append(
                        (dir, (self.position[0] + dx, self.position[1] + dy)))
                # 未発見であれば登録
                if self.mapping.mappingField.getTileInfo((self.position[0] + dx, self.position[1] + dy)) is None:
                    self.mapping.mappingField.registerTile(
                        (self.position[0] + dx, self.position[1] + dy))
        # 現在地のタイル情報を登録・更新
        self.mapping.mappingField.registerTile(
            (self.position[0], self.position[1]), incrementVisitTileCount=0)
        # 隣接壁情報を登録・更新
        self.mapping.mappingField.registerWall(
            (self.position[0], self.position[1]), neighborWalls)


        # ダイクストラ法で最も近い未到達タイルを探索
        unreached = self.mapping.dijkstra(
            self.position, self.direction, "nearestUnreached")
        # print("Unreached tiles:", unreached)
        if len(unreached) == 0:
            return True
        nearestTile = min(unreached.items(), key=lambda x: x[1].cost)[0]
        route = unreached[nearestTile].route
        # print("Nearest unreached tile:", nearestTile, "Route:", route)
        if len(route) < 2:
            return True
        print(self.mapping.mappingField)
        while len(route) > 1:
            nextPos = route[1]
            dx = nextPos[0] - self.position[0]
            dy = nextPos[1] - self.position[1]
            if dx == 1 and dy == 0:
                targetDir = 0
            elif dx == -1 and dy == 0:
                targetDir = 180
            elif dx == 0 and dy == -1:
                targetDir = 270
            elif dx == 0 and dy == 1:
                targetDir = 90
            else:
                print("Error: Invalid next position in route")
                return True
            turnAngle = (targetDir - self.direction + 360) % 360
            if turnAngle == 90:
                self.rotate(90)
            elif turnAngle == 180:
                self.rotate(90)
                self.rotate(90)
            elif turnAngle == 270:
                self.rotate(-90)
            self.move_forward()
            route.pop(0)
        return False