from . import mazeEnums
from . import mazeConstraints
from .device import deviceEnums
from config import get_logger
import heapq
import itertools
import copy
from typing import Callable
from dataclasses import dataclass

logger = get_logger(__name__)


def _turn_quarters(
    from_dir: mazeEnums.absDirection, to_dir: mazeEnums.absDirection
) -> int:
    """Return minimal number of 90-degree turns needed to rotate from from_dir to to_dir."""
    diff = (to_dir.value - from_dir.value) % 360
    diff = min(diff, (360 - diff) % 360)
    return int(diff // 90)


def dijkstra(
    mazeGraph: list[
        list[list[dict[mazeEnums.absDirection, tuple[int, int, int, float] | None]]]
    ],
    start: tuple[int, int, int],
    startDirection: mazeEnums.absDirection,
    goalCondition: Callable[[tuple[int, int, int]], bool],
    getTileType: Callable[[tuple[int, int, int]], mazeEnums.tileType],
) -> list[tuple[int, int, int]] | None:
    """Rotation-aware Dijkstra.

    State includes heading. Edge cost = (turn quarters * mazeConsrains.TURN_90_SEC) + mazeConsrains.MOVE_STRAIGHT_SEC.
    Returns a path as list of (x, y, z) positions.
    """

    start_state = (start[0], start[1], start[2], startDirection)
    dist: dict[tuple[int, int, int, mazeEnums.absDirection], float] = {start_state: 0.0}
    prev: dict[
        tuple[int, int, int, mazeEnums.absDirection],
        tuple[int, int, int, mazeEnums.absDirection] | None,
    ] = {start_state: None}

    # heapq compares tuple elements left-to-right. If earlier fields tie, it would
    # eventually compare absDirection, which isn't orderable -> TypeError.
    # Add a unique tiebreaker to guarantee comparability.
    push_id = itertools.count()
    heap: list[tuple[float, int, int, int, int, mazeEnums.absDirection]] = [
        (0.0, next(push_id), start[0], start[1], start[2], startDirection)
    ]

    while heap:
        cost, _, x, y, z, heading = heapq.heappop(heap)
        state = (x, y, z, heading)
        if cost != dist.get(state):
            continue

        pos = (x, y, z)
        if goalCondition(pos):
            logger.info(f"next goal: {pos}")
            # Reconstruct via state chain, then drop heading.
            states: list[tuple[int, int, int, mazeEnums.absDirection]] = []
            cur: tuple[int, int, int, mazeEnums.absDirection] | None = state
            while cur is not None:
                states.append(cur)
                cur = prev[cur]
            states.reverse()

            path: list[tuple[int, int, int]] = []
            for sx, sy, sz, _ in states:
                if not path or path[-1] != (sx, sy, sz):
                    path.append((sx, sy, sz))
            return path

        for move_dir in mazeEnums.absDirection:
            neighbor = mazeGraph[z][y][x][move_dir]
            if neighbor is None:
                continue
            nx, ny, nz, d = neighbor
            turn_q = _turn_quarters(heading, move_dir)
            step_cost = (turn_q * float(mazeConstraints.DIJKSTRA_COST_TURN_90_DEG)) + float(
                mazeConstraints.DIJKSTRA_COST_STRAIGHT_ONE_TILE * d / mazeConstraints.TILE_SIZE_CM
            )

            # 青タイルならコストを追加
            if getTileType((nx, ny, nz)) == mazeEnums.tileType.BLUE:
                step_cost += mazeConstraints.DIJKSTRA_COST_BLUE_TILE

            # 赤タイル -> Unknownの移動はコストを大きくする（先にDangerous以外を全部探索する）
            if (
                getTileType((nx, ny, nz)) == mazeEnums.tileType.UNKNOWN
                and getTileType(pos) == mazeEnums.tileType.RED
            ):
                step_cost += 10000

            new_cost = cost + step_cost

            new_state = (nx, ny, nz, move_dir)

            if new_cost < dist.get(new_state, float("inf")):
                dist[new_state] = new_cost
                prev[new_state] = state
                heapq.heappush(heap, (new_cost, next(push_id), nx, ny, nz, move_dir))

    return None


@dataclass
class layerInfoData:
    """レイヤーの情報構造体。"""

    isKnown: bool  # 発見済みのレイヤーかどうか
    layerNumber: int  # レイヤー番号（z座標）
    altitude: float  # レイヤー0からの高度差（cm）
    x_offset: float  # レイヤー0からのx方向のオフセット（cm）
    y_offset: float  # レイヤー0からのy方向のオフセット（cm）
    # offsetは、レイヤーxの座標にoffsetを足すと、レイヤー0に投影した座標になるような値

class mazeMap:
    def __init__(self, maxSize: int = 40, maxLayer: int = 10) -> None:
        self.maxSize = maxSize
        self.currentPosition: tuple[int, int, int] = (maxSize // 2, maxSize // 2, 0)
        self.maxLayer = maxLayer
        self.wallTypes = [
            [
                [
                    {d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection}
                    for _ in range(maxSize)
                ]
                for _ in range(maxSize)
            ]
            for _ in range(maxLayer)
        ]

        self.tileTypes = [
            [
                [mazeEnums.tileType.UNKNOWN for _ in range(maxSize)]
                for _ in range(maxSize)
            ]
            for _ in range(maxLayer)
        ]

        self.tileTypes[0][maxSize // 2][maxSize // 2] = mazeEnums.tileType.START

        # mazeAsGraph[z][y][x][d] : (z,y,x)から方向dに移動した時(nx,ny,nz, dist) distは移動先までの距離(cm), 壁がある場合はNone
        self.mazeAsGraph: list[
            list[list[dict[mazeEnums.absDirection, tuple[int, int, int, float] | None]]]
        ] = [
            [
                [{d: None for d in mazeEnums.absDirection} for _ in range(maxSize)]
                for _ in range(maxSize)
            ]
            for _ in range(maxLayer)
        ]
        self.frontDirection = mazeEnums.absDirection.NORTH

        self.seenVictimType = [
            [
                [{d: set() for d in mazeEnums.absDirection} for _ in range(maxSize)]
                for _ in range(maxSize)
            ]
            for _ in range(maxLayer)
        ]

        self.layerInfo: list[layerInfoData] = [
            layerInfoData(
                isKnown=False, layerNumber=i, altitude=0.0, x_offset=0.0, y_offset=0.0
            )
            for i in range(maxLayer)
        ]
        self.layerInfo[0] = layerInfoData(
            isKnown=True, layerNumber=0, altitude=0.0, x_offset=0.0, y_offset=0.0
        )
        self.knownLayerCount = 1  # すでに登録済みのレイヤー数

        self.nowRescueKitCount = mazeConstraints.DEFAULT_RESCUE_KIT_COUNT.copy()
        self.savedCache = dict()
        self.lastCheckpoint = self.currentPosition

        # 直前に上った坂の鉛直距離を3マス分保持
        self.movedVerticalDistanceNUM = 3
        self.movedVerticalDistance: list[tuple[float, bool]] = [
            (0, False) for _ in range(self.movedVerticalDistanceNUM)
        ]  # 1個前、2個前、3個前の坂の鉛直距離(cm)と坂検知をしたかのフラグ

        # 坂道検出をしたかどうかのフラグ
        self.isSlopeDetected = False

        # DangerousZone探索を始めたかどうかのフラグ
        self.isStartedDangerousZone = False

        self.isBrokenMapData = (
            False  # 壁検出エラーが発生して内部マップを破棄したかどうかのフラグ
        )
        self.startPosAfterBreakingMapData = (
            20,
            20,
        )  # マップデータを破壊後、スタート推定に使用。破壊場所と同じレイヤ（レイヤ0）にあったとしたときのX座標とY座標
        
        self.WallsAroundStartTile: dict[mazeEnums.absDirection, mazeEnums.wallType] = {d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection} # スタートタイル周辺の壁の情報。スタート位置推定に使用。

        self.saveCache()

    def setWallType(
        self, direction: mazeEnums.absDirection, wallType: mazeEnums.wallType
    ) -> None:
        x, y, z = self.currentPosition

        # もし壁がないならグラフを更新
        if wallType == mazeEnums.wallType.NO_WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                if self.tileTypes[z][y - 1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x][direction] = (
                        x,
                        y - 1,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
                    self.mazeAsGraph[z][y - 1][x][direction.opposite()] = (
                        x,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                if self.tileTypes[z][y][x + 1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x][direction] = (
                        x + 1,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
                    self.mazeAsGraph[z][y][x + 1][direction.opposite()] = (
                        x,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                if self.tileTypes[z][y + 1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x][direction] = (
                        x,
                        y + 1,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
                    self.mazeAsGraph[z][y + 1][x][direction.opposite()] = (
                        x,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                if self.tileTypes[z][y][x - 1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x][direction] = (
                        x - 1,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
                    self.mazeAsGraph[z][y][x - 1][direction.opposite()] = (
                        x,
                        y,
                        z,
                        mazeConstraints.TILE_SIZE_CM,
                    )
        elif wallType == mazeEnums.wallType.WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[z][y - 1][x][direction.opposite()] = None
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[z][y][x + 1][direction.opposite()] = None
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[z][y + 1][x][direction.opposite()] = None
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[z][y][x - 1][direction.opposite()] = None

        self.wallTypes[z][y][x][direction] = wallType

    def getWallType(
        self, direction: mazeEnums.absDirection | None = None
    ) -> dict[mazeEnums.absDirection, mazeEnums.wallType]:
        """
        @brief 現在位置の壁タイプを取得する
        @return: 現在位置の壁タイプの辞書
        """
        x, y, z = self.currentPosition
        if direction is not None:
            if direction == mazeEnums.absDirection.NORTH:
                return self.wallTypes[z][y - 1][x]
            elif direction == mazeEnums.absDirection.EAST:
                return self.wallTypes[z][y][x + 1]
            elif direction == mazeEnums.absDirection.SOUTH:
                return self.wallTypes[z][y + 1][x]
            elif direction == mazeEnums.absDirection.WEST:
                x = self.currentPosition[0] - 1
                return self.wallTypes[z][y][x - 1]
        return self.wallTypes[z][y][x]

    def getTileType(
        self, direction: mazeEnums.absDirection | None = None
    ) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y, z = self.currentPosition
        if direction is not None:
            neighbor = self.mazeAsGraph[z][y][x][direction]
            if neighbor is None:
                return mazeEnums.tileType.UNKNOWN
            nx, ny, nz, _ = neighbor
            return self.tileTypes[nz][ny][nx]
        return self.tileTypes[z][y][x]

    def isSeenVictimType(
        self,
        direction: list[mazeEnums.absDirection],
        victimType: deviceEnums.UnitVStatus,
    ) -> bool:
        """
        @brief 指定した方向に見えた被災者タイプが存在するか確認する
        @param direction: 被災者が見えた方向
        @param victimType: 確認する被災者タイプ
        @return: 指定した被災者タイプが見えた場合は True、そうでない場合は False
        """
        x, y, z = self.currentPosition
        return any(victimType in self.seenVictimType[z][y][x][d] for d in direction)

    def addSeenVictimType(
        self,
        direction: list[mazeEnums.absDirection],
        victimType: deviceEnums.UnitVStatus,
    ) -> None:
        """
        @brief 指定した方向に見えた被災者タイプを追加する
        @param direction: 被災者が見えた方向
        @param victimType: 追加する被災者タイプ
        """
        x, y, z = self.currentPosition
        for d in direction:
            self.seenVictimType[z][y][x][d].add(victimType)

    def getSeenVictimType(
        self, x: int, y: int, z: int
    ) -> dict[mazeEnums.absDirection, set[deviceEnums.UnitVStatus]]:
        """
        @brief 見た被災者タイプを取得する
        @return: 見た被災者タイプの辞書
        """
        return self.seenVictimType[z][y][x]

    def setTileType(
        self,
        tiletype: mazeEnums.tileType,
        direction: mazeEnums.absDirection | None = None,
    ) -> None:
        """
        @brief 指定した方向のタイルタイプを設定する。directionがNoneの場合は現在位置に設定する
        @param tiletype: 設定するタイルタイプ
        @param direction: 設定する方向
        """
        x, y, z = self.currentPosition
        if direction is None:
            pass
        else:
            if direction == mazeEnums.absDirection.NORTH:
                y -= 1
            elif direction == mazeEnums.absDirection.EAST:
                x += 1
            elif direction == mazeEnums.absDirection.SOUTH:
                y += 1
            elif direction == mazeEnums.absDirection.WEST:
                x -= 1
        assert (
            0 <= x < self.maxSize and 0 <= y < self.maxSize and 0 <= z < self.maxLayer
        ), self.renderKnownTileAndWall()

        self.tileTypes[z][y][x] = tiletype

        if tiletype == mazeEnums.tileType.BLACK:
            # 黒タイルならその周囲の通路を塞ぐ
            for direction in mazeEnums.absDirection:
                if direction == mazeEnums.absDirection.NORTH and y > 0:
                    self.mazeAsGraph[z][y][x][direction] = None
                    self.mazeAsGraph[z][y - 1][x][direction.opposite()] = None
                elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                    self.mazeAsGraph[z][y][x][direction] = None
                    self.mazeAsGraph[z][y][x + 1][direction.opposite()] = None
                elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                    self.mazeAsGraph[z][y][x][direction] = None
                    self.mazeAsGraph[z][y + 1][x][direction.opposite()] = None
                elif direction == mazeEnums.absDirection.WEST and x > 0:
                    self.mazeAsGraph[z][y][x][direction] = None
                    self.mazeAsGraph[z][y][x - 1][direction.opposite()] = None

        if tiletype == mazeEnums.tileType.SILVER:
            logger.debug("Silver tile detected, saving cache")
            self.saveCache()

    def existsLayerWithinAltitude(
        self, altitude: float, tolerance: float = 7.0
    ) -> int | None:
        """
        @brief 指定した高度に近いレイヤーが存在するか確認する
        @param altitude: 確認する高度（cm）
        @param tolerance: 高度の許容誤差（cm）
        @return: 近いレイヤーが存在する場合はそのレイヤー番号、存在しない場合は None
        """
        for layerNumber, info in enumerate(self.layerInfo):
            if info.isKnown and abs(info.altitude - altitude) <= tolerance:
                return layerNumber
        return None

    def getDistanceToNextTile(self, direction: mazeEnums.absDirection) -> float | None:
        x, y, z = self.currentPosition
        neighbor = self.mazeAsGraph[z][y][x][direction]
        if neighbor is None:
            return None
        _, _, _, d = neighbor
        return d

    def setSlope(
        self,
        direction: mazeEnums.absDirection,
        horizontalDistance: float,
        verticalDistance: float,
    ) -> None:
        """
        @brief 指定した方向に坂があると設定する
        @param direction: 坂がある方向
        @param horizontalDistance: 水平距離 (cm)
        @param verticalDistance: 垂直距離 (cm)
        """
        x, y, z = self.currentPosition
        if horizontalDistance != 0:
            # horizontalDistanceが30cmの倍数になるように調整
            new_horizontalDistance = (
                round(horizontalDistance / mazeConstraints.TILE_SIZE_CM)
                * mazeConstraints.TILE_SIZE_CM
            )
            verticalDistance = (
                verticalDistance * (new_horizontalDistance / horizontalDistance)
                if horizontalDistance != 0
                else verticalDistance
            )
            horizontalDistance = new_horizontalDistance

        nextLayerAltitude = self.layerInfo[z].altitude + verticalDistance
        existingLayer = self.existsLayerWithinAltitude(nextLayerAltitude)
        if existingLayer is not None:
            nextLayer = existingLayer
            x_offset = self.layerInfo[z].x_offset - self.layerInfo[nextLayer].x_offset
            y_offset = self.layerInfo[z].y_offset - self.layerInfo[nextLayer].y_offset
            if direction == mazeEnums.absDirection.NORTH:
                y_offset -= horizontalDistance
                y_offset -= mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.EAST:
                x_offset += horizontalDistance
                x_offset += mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.SOUTH:
                y_offset += horizontalDistance
                y_offset += mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.WEST:
                x_offset -= horizontalDistance
                x_offset -= mazeConstraints.TILE_SIZE_CM
            # マス数に変換
            x_offset_tiles = round(x_offset / mazeConstraints.TILE_SIZE_CM)
            y_offset_tiles = round(y_offset / mazeConstraints.TILE_SIZE_CM)
            next_x = x + x_offset_tiles
            next_y = y + y_offset_tiles
            assert (
                0 <= next_x < self.maxSize and 0 <= next_y < self.maxSize
            ), f"Calculated layer offset leads to out-of-bounds position: ({next_x}, {next_y})"

            # グラフ情報更新
            ## direction方向のグラフを削除
            if self.mazeAsGraph[z][y][x][direction] is not None:
                _x, _y, _z, _d = self.mazeAsGraph[z][y][x][direction]  # type: ignore
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[_z][_y][_x][direction.opposite()] = None

            self.mazeAsGraph[nextLayer][next_y][next_x][direction.opposite()] = None

            ## 新しいレイヤーのグラフを追加
            self.mazeAsGraph[z][y][x][direction] = (
                next_x,
                next_y,
                nextLayer,
                horizontalDistance + mazeConstraints.TILE_SIZE_CM,
            )
            self.mazeAsGraph[nextLayer][next_y][next_x][direction.opposite()] = (
                x,
                y,
                z,
                horizontalDistance + mazeConstraints.TILE_SIZE_CM,
            )
            self.wallTypes[nextLayer][next_y][next_x][
                direction.opposite()
            ] = mazeEnums.wallType.NO_WALL

        else:
            nextLayer = self.knownLayerCount
            new_x_offset = self.layerInfo[z].x_offset
            new_y_offset = self.layerInfo[z].y_offset
            if direction == mazeEnums.absDirection.NORTH:
                new_y_offset -= horizontalDistance
                new_y_offset -= mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.EAST:
                new_x_offset += horizontalDistance
                new_x_offset += mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.SOUTH:
                new_y_offset += horizontalDistance
                new_y_offset += mazeConstraints.TILE_SIZE_CM
            elif direction == mazeEnums.absDirection.WEST:
                new_x_offset -= horizontalDistance
                new_x_offset -= mazeConstraints.TILE_SIZE_CM
            self.layerInfo[nextLayer] = layerInfoData(
                isKnown=True,
                layerNumber=nextLayer,
                altitude=nextLayerAltitude,
                x_offset=new_x_offset,
                y_offset=new_y_offset,
            )
            self.knownLayerCount += 1

            # グラフ情報更新
            ## direction方向のグラフを削除
            if self.mazeAsGraph[z][y][x][direction] is not None:
                _x, _y, _z, _d = self.mazeAsGraph[z][y][x][direction]  # type: ignore
                self.mazeAsGraph[z][y][x][direction] = None
                self.mazeAsGraph[_z][_y][_x][direction.opposite()] = None

            ## 新しいレイヤーのグラフを追加
            self.mazeAsGraph[z][y][x][direction] = (
                x,
                y,
                nextLayer,
                horizontalDistance + mazeConstraints.TILE_SIZE_CM,
            )
            self.mazeAsGraph[nextLayer][y][x][direction.opposite()] = (
                x,
                y,
                z,
                horizontalDistance + mazeConstraints.TILE_SIZE_CM,
            )

            # WallTypeも更新
            self.wallTypes[nextLayer][y][x][
                direction.opposite()
            ] = mazeEnums.wallType.NO_WALL

    def setMovedVerticalDistance(self, distance: float, isSlopeDetected: bool) -> None:
        """
        @brief 直前に移動した坂の鉛直距離を更新する
        @param distance: 直前に移動した坂の鉛直距離（cm）
        @param isSlopeDetected: 坂検知をしたかのフラグ
        """
        for i in range(self.movedVerticalDistanceNUM - 1, 0, -1):
            self.movedVerticalDistance[i] = self.movedVerticalDistance[i - 1]
        self.movedVerticalDistance[0] = (distance, isSlopeDetected)

    def getMovedVerticalDistance(self) -> list[tuple[float, bool]]:
        """
        @brief 直前に移動した坂の鉛直距離を取得する
        @return: 直前に移動した坂の鉛直距離のリスト（cm）と坂検知をしたかのフラグ
        """
        return self.movedVerticalDistance[:]

    def resetMovedVerticalDistance(self) -> None:
        """
        @brief 直前に移動した坂の鉛直距離をリセットする
        """
        self.movedVerticalDistance = [(0, False) for _ in range(self.movedVerticalDistanceNUM)]

    # def getAroundTileType(self) -> dict[mazeEnums.absDirection, mazeEnums.tileType]:
    #     """
    #     @brief 現在位置の周囲のタイルタイプを取得する
    #     @return: 周囲のタイルタイプの辞書
    #     """
    #     x, y = self.currentPosition
    #     aroundTiles = {d: mazeEnums.tileType.UNKNOWN for d in mazeEnums.absDirection}

    #     if y > 0:
    #         aroundTiles[mazeEnums.absDirection.NORTH] = self.tileTypes[y-1][x]
    #     if x < self.maxSize - 1:
    #         aroundTiles[mazeEnums.absDirection.EAST] = self.tileTypes[y][x+1]
    #     if y < self.maxSize - 1:
    #         aroundTiles[mazeEnums.absDirection.SOUTH] = self.tileTypes[y+1][x]
    #     if x > 0:
    #         aroundTiles[mazeEnums.absDirection.WEST] = self.tileTypes[y][x-1]

    #     return aroundTiles

    # def getCurrentTileType(self) -> mazeEnums.tileType:
    #     """
    #     @brief 現在位置のタイルタイプを取得する
    #     @return: 現在位置のタイルタイプ
    #     """
    #     x, y = self.currentPosition
    #     return self.tileTypes[y][x]
    
    def getCostToStartTile(self) -> float:
        """
        @brief スタートタイルまでのコストを取得する
        @return: スタートタイルまでのコスト(SEC)。到達不可能な場合は 0 を返す
        """
        x, y, z = self.currentPosition
        
        start_tile_position = self.estimateStartTile()
        path = dijkstra(
            self.mazeAsGraph,
            (x, y, z),
            self.frontDirection,
            lambda pos: pos == start_tile_position, 
            lambda pos: self.tileTypes[pos[2]][pos[1]][pos[0]],
        )

        if path is None:
            return 0.0

        cost = 0.0
        for i in range(1, len(path)):
            currX, currY, currZ = path[i - 1]
            nextX, nextY, nextZ = path[i]
            for direction in mazeEnums.absDirection:
                neighbor = self.mazeAsGraph[currZ][currY][currX][direction]
                if neighbor is None:
                    continue
                nx, ny, nz, d = neighbor
                if (nx, ny, nz) == (nextX, nextY, nextZ):
                    turn_q = _turn_quarters(self.frontDirection, direction)
                    step_cost = (turn_q * float(mazeConstraints.DIJKSTRA_COST_TURN_90_DEG)) + float(
                        mazeConstraints.DIJKSTRA_COST_STRAIGHT_ONE_TILE * d / mazeConstraints.TILE_SIZE_CM
                    )
                    cost += step_cost
                    self.frontDirection = direction
                    break
        return cost

    def getNearestUnexploredTile(self) -> list[mazeEnums.absDirection] | None:
        """
        @brief 最も近い未探索タイルへのパスを取得する
        @return: 未探索タイルへの方向リスト。未探索タイルが存在しない場合は None を返す
        """
        x, y, z = self.currentPosition
        path = dijkstra(
            self.mazeAsGraph,
            (x, y, z),
            self.frontDirection,
            lambda pos: pos != (x, y, z)
            and self.tileTypes[pos[2]][pos[1]][pos[0]] == mazeEnums.tileType.UNKNOWN,
            lambda pos: self.tileTypes[pos[2]][pos[1]][pos[0]],
        )

        if path is None:
            return None

        directions = []

        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, currZ = path[i - 1]
            nextX, nextY, nextZ = path[i]
            for direction in mazeEnums.absDirection:
                neighbor = self.mazeAsGraph[currZ][currY][currX][direction]
                if neighbor is None:
                    continue
                nx, ny, nz, _d = neighbor
                if (nx, ny, nz) == (nextX, nextY, nextZ):
                    directions.append(direction)
                    break

        return directions

    def getPathTo(
        self, target: tuple[int, int, int]
    ) -> list[mazeEnums.absDirection] | None:
        """
        @brief 指定した座標へのパスを取得する
        @param target: 目的地の座標 (x, y, z)
        @return: 目的地への方向リスト。到達不可能な場合は None を返す
        """
        x, y, z = self.currentPosition
        path = dijkstra(
            self.mazeAsGraph,
            (x, y, z),
            self.frontDirection,
            lambda pos: pos == target,
            lambda pos: self.tileTypes[pos[2]][pos[1]][pos[0]],
        )

        if path is None:
            return None

        directions = []

        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, currZ = path[i - 1]
            nextX, nextY, nextZ = path[i]
            for direction in mazeEnums.absDirection:
                neighbor = self.mazeAsGraph[currZ][currY][currX][direction]
                if neighbor is None:
                    continue
                nx, ny, nz, _d = neighbor
                if (nx, ny, nz) == (nextX, nextY, nextZ):
                    directions.append(direction)
                    break

        return directions

    def moveTo(self, direction: mazeEnums.absDirection) -> None:
        """
        @brief 指定した方向に移動する
        @param direction: 移動する方向
        """
        x, y, z = self.currentPosition
        if self.wallTypes[z][y][x][direction] != mazeEnums.wallType.NO_WALL:
            logger.warning(
                f"Moving to wall direction: {direction}\n{self.renderKnownTileAndWall()}"
            )
        self.currentPosition = self.mazeAsGraph[z][y][x][direction][0:3]  # type: ignore

        # 赤タイルからUnknownタイルへ移動した時、isStartedDangerousZoneをTrueにする
        if (
            self.tileTypes[z][y][x] == mazeEnums.tileType.RED
            and self.tileTypes[self.currentPosition[2]][self.currentPosition[1]][
                self.currentPosition[0]
            ]
            == mazeEnums.tileType.UNKNOWN
        ):
            self.isStartedDangerousZone = True

        # if direction == mazeEnums.absDirection.NORTH:
        #     self.currentPosition = (x, y - 1, z)
        # elif direction == mazeEnums.absDirection.EAST:
        #     self.currentPosition = (x + 1, y, z)
        # elif direction == mazeEnums.absDirection.SOUTH:
        #     self.currentPosition = (x, y + 1, z)
        # elif direction == mazeEnums.absDirection.WEST:
        #     self.currentPosition = (x - 1, y, z)
        self.updateFrontDirection(direction)

    def updateFrontDirection(self, direction: mazeEnums.absDirection) -> None:
        """
        @brief 前方方向を設定
        @param direction: 設定する方向
        """
        self.frontDirection = direction

    def dropRescueKit(self, side: deviceEnums.Side, count: int) -> None:
        """
        @brief 指定したサイドからレスキューキットを落とす
        @param side: レスキューキットを落とすサイド
        @param count: 落とすレスキューキットの数
        """
        assert self.nowRescueKitCount[side] >= count, "Not enough rescue kits to drop."
        self.nowRescueKitCount[side] -= count

    def saveCache(self) -> None:
        """
        @brief 現在のマップ状態をキャッシュに保存する
        """
        self.savedCache["tileTypes"] = [
            [copy.deepcopy(row) for row in self.tileTypes[z]]
            for z in range(self.maxLayer)
        ]
        self.savedCache["wallTypes"] = [
            [
                [{d: wt[d] for d in mazeEnums.absDirection} for wt in row]
                for row in self.wallTypes[z]
            ]
            for z in range(self.maxLayer)
        ]
        self.savedCache["mazeAsGraph"] = [
            [
                [copy.deepcopy(neighbors) for neighbors in row]
                for row in self.mazeAsGraph[z]
            ]
            for z in range(self.maxLayer)
        ]

        self.savedCache["layerInfo"] = copy.deepcopy(self.layerInfo)
        self.savedCache["knownLayerCount"] = self.knownLayerCount
        self.lastCheckpoint = self.currentPosition
        self.savedCache["isSlopeDetected"] = self.isSlopeDetected
        self.savedCache["isStartedDangerousZone"] = self.isStartedDangerousZone
        self.savedCache["seenVictimType"] = [
            [
                [{d: set(vt) for d, vt in cell.items()} for cell in row]
                for row in self.seenVictimType[z]
            ]
            for z in range(self.maxLayer)
        ]
        self.savedCache["isBrokenMapData"] = self.isBrokenMapData

    def loadCache(self, nowDirection: mazeEnums.absDirection) -> None:
        """
        @brief キャッシュからマップ状態を復元する
        @param nowDirection: 現在の前方方向
        """
        if "tileTypes" in self.savedCache and "wallTypes" in self.savedCache:
            self.tileTypes = [
                [copy.deepcopy(row) for row in self.savedCache["tileTypes"][z]]
                for z in range(self.maxLayer)
            ]
            self.wallTypes = [
                [
                    [{d: wt[d] for d in mazeEnums.absDirection} for wt in row]
                    for row in self.savedCache["wallTypes"][z]
                ]
                for z in range(self.maxLayer)
            ]

            self.mazeAsGraph = [
                [
                    [copy.deepcopy(neighbors) for neighbors in row]
                    for row in self.savedCache["mazeAsGraph"][z]
                ]
                for z in range(self.maxLayer)
            ]
            self.frontDirection = nowDirection
            self.currentPosition = self.lastCheckpoint
            self.layerInfo = copy.deepcopy(self.savedCache["layerInfo"])
            self.knownLayerCount = self.savedCache["knownLayerCount"]
            self.isSlopeDetected = self.savedCache["isSlopeDetected"]
            self.isStartedDangerousZone = self.savedCache["isStartedDangerousZone"]
            self.seenVictimType = [
                [
                    [{d: set(vt) for d, vt in cell.items()} for cell in row]
                    for row in self.savedCache["seenVictimType"][z]
                ]
                for z in range(self.maxLayer)
            ]
            self.movedVerticalDistance = [
                (0, False) for _ in range(self.movedVerticalDistanceNUM)
            ]
            self.isBrokenMapData = self.savedCache["isBrokenMapData"]
            

    def resetMapData(self) -> None:
        """
        @brief マップデータを初期状態にリセットする。キャッシュは保持する。
        """
        if self.WallsAroundStartTile == {d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection}: # スタートタイル周辺の壁の情報が未取得の場合、情報を保存
            for d in mazeEnums.absDirection:
                self.WallsAroundStartTile[d] = self.wallTypes[0][self.maxSize // 2][self.maxSize // 2][d]
                
        self.frontDirection = mazeEnums.absDirection.NORTH
        self.tileTypes = [
            [
                [mazeEnums.tileType.UNKNOWN for _ in range(self.maxSize)]
                for _ in range(self.maxSize)
            ]
            for _ in range(self.maxLayer)
        ]


        self.wallTypes = [
            [
                [
                    {d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection}
                    for _ in range(self.maxSize)
                ]
                for _ in range(self.maxSize)
            ]
            for _ in range(self.maxLayer)
        ]

        self.mazeAsGraph = [
            [
                [{d: None for d in mazeEnums.absDirection} for _ in range(self.maxSize)]
                for _ in range(self.maxSize)
            ]
            for _ in range(self.maxLayer)
        ]

        self.seenVictimType = [
            [
                [
                    {d: set() for d in mazeEnums.absDirection}
                    for _ in range(self.maxSize)
                ]
                for _ in range(self.maxSize)
            ]
            for _ in range(self.maxLayer)
        ]

        self.layerInfo = [
            layerInfoData(
                isKnown=False, layerNumber=i, altitude=0.0, x_offset=0.0, y_offset=0.0
            )
            for i in range(self.maxLayer)
        ]
        self.layerInfo[0] = layerInfoData(
            isKnown=True, layerNumber=0, altitude=0.0, x_offset=0.0, y_offset=0.0
        )
        self.knownLayerCount = 1

        self.movedVerticalDistance = [
            (0, False) for _ in range(self.movedVerticalDistanceNUM)
        ]

        self.isSlopeDetected = False
        self.isStartedDangerousZone = False
        self.isBrokenMapData = True
        
        # スタート位置の推定に使用する値を、マップデータを破壊したときの位置に合わせて更新する
        old_start_x, old_start_y = self.startPosAfterBreakingMapData
        _,_,current_z = self.currentPosition
        offset_x, offset_y = self.layerInfo[current_z].x_offset, self.layerInfo[current_z].y_offset
        self.startPosAfterBreakingMapData = (old_start_x - offset_x, old_start_y - offset_y)


    def estimateStartTile(self) -> tuple[int, int, int]:
        """Startタイルの位置を推定する。マップが壊れていなければ初期位置（20,20,0）を返す。
        マップが壊れている場合は、初期位置との相対位置をもとに、壁情報が一致する位置を探索する。見つからない場合は現在位置を返す。
        """
        if not self.isBrokenMapData:
            return (self.maxSize // 2, self.maxSize // 2, 0)


        # 全てのレイヤー、スタート位置＋周囲4方向のタイルを探索
        for z in range(self.knownLayerCount):
            nx,ny = self.startPosAfterBreakingMapData
            nx -= self.layerInfo[z].x_offset
            ny -= self.layerInfo[z].y_offset
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if dx != 0 and dy != 0:
                        continue
                    x = int(nx + dx)
                    y = int(ny + dy)
                    if not (0 <= x < self.maxSize and 0 <= y < self.maxSize):
                        continue
                    if self.tileTypes[z][y][x] == mazeEnums.tileType.UNKNOWN:
                        continue

                    # 壁情報が一致するか確認
                    match = True
                    for d in mazeEnums.absDirection:
                        if self.wallTypes[z][y][x][d] != self.WallsAroundStartTile[d]:
                            match = False
                            break
                        
                    # 周囲4つのマスの壁情報も一致するか確認
                    for d in mazeEnums.absDirection:
                        nx, ny = x, y
                        if d == mazeEnums.absDirection.NORTH:
                            ny -= 1
                        elif d == mazeEnums.absDirection.EAST:
                            nx += 1
                        elif d == mazeEnums.absDirection.SOUTH:
                            ny += 1
                        elif d == mazeEnums.absDirection.WEST:
                            nx -= 1
                        if not (0 <= nx < self.maxSize and 0 <= ny < self.maxSize):
                            continue
                        if self.tileTypes[z][ny][nx] == mazeEnums.tileType.UNKNOWN:
                            continue
                        for nd in mazeEnums.absDirection:
                            if self.wallTypes[z][ny][nx][nd] != self.WallsAroundStartTile[nd]:
                                match = False
                                break
                        if not match:
                            break
                    if match:
                        logger.debug(f"Estimated start tile at ({x}, {y}, {z}) based on wall information.")
                        return (x, y, z)


        # 見つからない場合は現在位置を返す
        return self.currentPosition

    def _is_known_cell(self, x: int, y: int, z: int) -> bool:
        if self.tileTypes[z][y][x] != mazeEnums.tileType.UNKNOWN:
            return True
        wt = self.wallTypes[z][y][x]
        return any(v != mazeEnums.wallType.UNKNOWN for v in wt.values())

    def _get_known_bounds(self, z: int) -> tuple[int, int, int, int]:
        """known な情報(タイル/壁)が存在する範囲に切り詰めた bbox を返す。

        戻り値: (min_x, min_y, max_x, max_y) いずれも inclusive。
        """
        min_x = self.maxSize
        min_y = self.maxSize
        max_x = -1
        max_y = -1

        for y in range(self.maxSize):
            for x in range(self.maxSize):
                if self._is_known_cell(x, y, z):
                    if x < min_x:
                        min_x = x
                    if y < min_y:
                        min_y = y
                    if x > max_x:
                        max_x = x
                    if y > max_y:
                        max_y = y

        # 少なくとも START が known のはずだが、念のため。
        if max_x < 0:
            cx, cy, cz = self.currentPosition
            return (
                cx,
                cy,
                cx,
                cy,
            )
        return (
            min_x,
            min_y,
            max_x,
            max_y,
        )

    def _wall_as_bool(self, wall: mazeEnums.wallType) -> bool:
        """表示用: wall / noWall の2分類。UNKNOWNは表示上 noWall と同等に扱う。"""
        if wall == mazeEnums.wallType.NO_WALL:
            return False
        if wall == mazeEnums.wallType.WALL:
            return True
        # victim などは壁扱い
        if wall != mazeEnums.wallType.UNKNOWN:
            return True
        return False

    def renderKnownTileAndWall(self) -> str:
        """tileTypes と wallTypes をまとめて、UNKNOWNのみの行列が出ない範囲でASCII表示する。"""

        def tile_char(x: int, y: int, z: int) -> str:
            if (x, y, z) == self.currentPosition:
                return "@"
            t = self.tileTypes[z][y][x]
            # tileType は __str__ 実装済み(U/E/R/...)。
            return str(t)

        def wall_at(x: int, y: int, z: int, d: mazeEnums.absDirection) -> bool:
            """(x,y) の d 方向の壁を、両側タイルの情報で OR 判定して返す。

            片側だけ更新されて不整合が起きても、「どちらかが壁」なら壁として描画する。
            """
            w1 = self.wallTypes[z][y][x][d]

            nx, ny = x, y
            od: mazeEnums.absDirection | None = None
            if d == mazeEnums.absDirection.NORTH:
                nx, ny = x, y - 1
                od = mazeEnums.absDirection.SOUTH
            elif d == mazeEnums.absDirection.EAST:
                nx, ny = x + 1, y
                od = mazeEnums.absDirection.WEST
            elif d == mazeEnums.absDirection.SOUTH:
                nx, ny = x, y + 1
                od = mazeEnums.absDirection.NORTH
            elif d == mazeEnums.absDirection.WEST:
                nx, ny = x - 1, y
                od = mazeEnums.absDirection.EAST

            b1 = self._wall_as_bool(w1)
            if od is None or not (0 <= nx < self.maxSize and 0 <= ny < self.maxSize):
                return b1

            w2 = self.wallTypes[z][ny][nx][od]
            b2 = self._wall_as_bool(w2)
            return b1 or b2

        lines: list[str] = []
        cx, cy, cz = self.currentPosition
        lines.append(
            f"Current position: ({cx}, {cy}, {cz}), front direction: {self.frontDirection}"
        )

        for z in range(self.knownLayerCount):
            min_x, min_y, max_x, max_y = self._get_known_bounds(z)
            lines.append(
                f"Layer {z} (altitude={self.layerInfo[z].altitude}cm, offset=({self.layerInfo[z].x_offset}cm, {self.layerInfo[z].y_offset}cm)):"
            )
            lines.append(f"Known map area: x={min_x}..{max_x}, y={min_y}..{max_y}")

            # 上端(NORTH)
            top = ["+"]
            for x in range(min_x, max_x + 1):
                top.append(
                    "---"
                    if wall_at(x, min_y, z, mazeEnums.absDirection.NORTH)
                    else "   "
                )
                top.append("+")
            lines.append("".join(top))

            for y in range(min_y, max_y + 1):
                row = []
                # 左端(WEST)
                row.append(
                    "|" if wall_at(min_x, y, z, mazeEnums.absDirection.WEST) else " "
                )
                for x in range(min_x, max_x + 1):
                    row.append(f" {tile_char(x, y, z)} ")
                    row.append(
                        "|" if wall_at(x, y, z, mazeEnums.absDirection.EAST) else " "
                    )
                lines.append("".join(row))

                # 下端(SOUTH)
                sep = ["+"]
                for x in range(min_x, max_x + 1):
                    sep.append(
                        "---"
                        if wall_at(x, y, z, mazeEnums.absDirection.SOUTH)
                        else "   "
                    )
                    sep.append("+")
                lines.append("".join(sep))
        # 既知のすべてのマスについてmazeAsGraphの値を出力
        for z in range(self.knownLayerCount):
            for y in range(self.maxSize):
                for x in range(self.maxSize):
                    if self._is_known_cell(x, y, z):
                        neighbors = self.mazeAsGraph[z][y][x]
                        lines.append(
                            f"mazeAsGraph[{z}][{y}][{x}] = {{"
                            + ", ".join(
                                f"{d}: {neighbors[d]}" for d in mazeEnums.absDirection
                            )
                            + "}"
                        )
        return "\n".join(lines)
