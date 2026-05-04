from . import mazeEnums
from . import mazeConstraints
from .device import deviceEnums
from .device.arduinoNanoEvery import ArduinoNanoEveryUART, _NullArduinoNanoEveryUART
from config import get_logger
import heapq
import itertools
import copy
from typing import Callable

logger = get_logger(__name__)


def _turn_quarters(
    from_dir: mazeEnums.absDirection, to_dir: mazeEnums.absDirection
) -> int:
    """Return minimal number of 90-degree turns needed to rotate from from_dir to to_dir."""
    diff = (to_dir.value - from_dir.value) % 360
    diff = min(diff, (360 - diff) % 360)
    return int(diff // 90)


def _step_direction(
    current: tuple[int, int, int],
    neighbor: tuple[int, int, int],
) -> mazeEnums.absDirection:
    cx, cy, cz = current
    nx, ny, nz = neighbor
    dx = nx - cx
    dy = ny - cy
    if dx == 0 and dy == -1:
        return mazeEnums.absDirection.NORTH
    if dx == 1 and dy == 0:
        return mazeEnums.absDirection.EAST
    if dx == 0 and dy == 1:
        return mazeEnums.absDirection.SOUTH
    if dx == -1 and dy == 0:
        return mazeEnums.absDirection.WEST
    raise ValueError(f"neighbor must be adjacent: {current} -> {neighbor}")


def dijkstra(
    mazeGraph: list[list[list[set[tuple[int, int, int]]]]],
    start: tuple[int, int, int],
    startDirection: mazeEnums.absDirection,
    goalCondition: Callable[[tuple[int, int, int]], bool],
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

        for neighbor in mazeGraph[z][y][x]:
            move_dir = _step_direction(pos, neighbor)
            turn_q = _turn_quarters(heading, move_dir)
            step_cost = (turn_q * float(mazeConstraints.TURN_90_SEC)) + float(
                mazeConstraints.MOVE_STRAIGHT_SEC
            )
            new_cost = cost + step_cost
            nx, ny, nz = neighbor
            new_state = (nx, ny, nz, move_dir)

            if new_cost < dist.get(new_state, float("inf")):
                dist[new_state] = new_cost
                prev[new_state] = state
                heapq.heappush(heap, (new_cost, next(push_id), nx, ny, nz, move_dir))

    return None


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
        self.mazeAsGraph = [
            [[set() for _ in range(maxSize)] for _ in range(maxSize)]
            for _ in range(maxLayer)
        ]
        self.frontDirection = mazeEnums.absDirection.NORTH
        self.arduinoNanoEvery: ArduinoNanoEveryUART | _NullArduinoNanoEveryUART = (
            _NullArduinoNanoEveryUART()
        )
        self.seenVictimType = [
            [
                [{d: set() for d in mazeEnums.absDirection} for _ in range(maxSize)]
                for _ in range(maxSize)
            ]
            for _ in range(maxLayer)
        ]

        try:
            self.arduinoNanoEvery = ArduinoNanoEveryUART(port="/dev/ttyUSB0")
        except Exception as exc:
            logger.warning(f"ArduinoNanoEveryUART init failed: {exc}")
        self.nowRescueKitCount = mazeConstraints.DEFAULT_RESCUE_KIT_COUNT.copy()
        self.savedCache = dict()
        self.lastCheckpoint = self.currentPosition
        self.saveCache()
        self.updateArduinoStatus()

    def setWallType(
        self, direction: mazeEnums.absDirection, wallType: mazeEnums.wallType
    ) -> None:
        x, y, z = self.currentPosition

        # もし壁がないならグラフを更新
        if wallType == mazeEnums.wallType.NO_WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                if self.tileTypes[z][y - 1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x].add((x, y - 1, z))
                    self.mazeAsGraph[z][y - 1][x].add((x, y, z))
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                if self.tileTypes[z][y][x + 1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x].add((x + 1, y, z))
                    self.mazeAsGraph[z][y][x + 1].add((x, y, z))
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                if self.tileTypes[z][y + 1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x].add((x, y + 1, z))
                    self.mazeAsGraph[z][y + 1][x].add((x, y, z))
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                if self.tileTypes[z][y][x - 1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[z][y][x].add((x - 1, y, z))
                    self.mazeAsGraph[z][y][x - 1].add((x, y, z))
        elif wallType == mazeEnums.wallType.WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                self.mazeAsGraph[z][y][x].discard((x, y - 1, z))
                self.mazeAsGraph[z][y - 1][x].discard((x, y, z))
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                self.mazeAsGraph[z][y][x].discard((x + 1, y, z))
                self.mazeAsGraph[z][y][x + 1].discard((x, y, z))
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                self.mazeAsGraph[z][y][x].discard((x, y + 1, z))
                self.mazeAsGraph[z][y + 1][x].discard((x, y, z))
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                self.mazeAsGraph[z][y][x].discard((x - 1, y, z))
                self.mazeAsGraph[z][y][x - 1].discard((x, y, z))

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

    def getTileType(self) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y, z = self.currentPosition
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
                    self.mazeAsGraph[z][y][x].discard((x, y - 1, z))
                    self.mazeAsGraph[z][y - 1][x].discard((x, y, z))
                elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                    self.mazeAsGraph[z][y][x].discard((x + 1, y, z))
                    self.mazeAsGraph[z][y][x + 1].discard((x, y, z))
                elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                    self.mazeAsGraph[z][y][x].discard((x, y + 1, z))
                    self.mazeAsGraph[z][y + 1][x].discard((x, y, z))
                elif direction == mazeEnums.absDirection.WEST and x > 0:
                    self.mazeAsGraph[z][y][x].discard((x - 1, y, z))
                    self.mazeAsGraph[z][y][x - 1].discard((x, y, z))

        if tiletype == mazeEnums.tileType.SILVER:
            logger.debug("Silver tile detected, saving cache")
            self.saveCache()

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
            and self.tileTypes[pos[1]][pos[0]] == mazeEnums.tileType.UNKNOWN,
        )

        if path is None:
            return None

        directions = []

        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, currZ = path[i - 1]
            nextX, nextY, nextZ = path[i]
            if nextX == currX and nextY == currY - 1:
                directions.append(mazeEnums.absDirection.NORTH)
            elif nextX == currX + 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.EAST)
            elif nextX == currX and nextY == currY + 1:
                directions.append(mazeEnums.absDirection.SOUTH)
            elif nextX == currX - 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.WEST)

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
            self.mazeAsGraph, (x, y, z), self.frontDirection, lambda pos: pos == target
        )

        if path is None:
            return None

        directions = []

        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, currZ = path[i - 1]
            nextX, nextY, nextZ = path[i]
            if nextX == currX and nextY == currY - 1:
                directions.append(mazeEnums.absDirection.NORTH)
            elif nextX == currX + 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.EAST)
            elif nextX == currX and nextY == currY + 1:
                directions.append(mazeEnums.absDirection.SOUTH)
            elif nextX == currX - 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.WEST)

        return directions

    def moveTo(self, direction: mazeEnums.absDirection) -> None:
        """
        @brief 指定した方向に移動する
        @param direction: 移動する方向
        """
        x, y, z = self.currentPosition
        if self.wallTypes[z][y][x][direction] != mazeEnums.wallType.NO_WALL:
            logger.debug(
                f"Moving to wall direction: {direction}\n{self.renderKnownTileAndWall()}"
            )

        if direction == mazeEnums.absDirection.NORTH:
            self.currentPosition = (x, y - 1, z)
        elif direction == mazeEnums.absDirection.EAST:
            self.currentPosition = (x + 1, y, z)
        elif direction == mazeEnums.absDirection.SOUTH:
            self.currentPosition = (x, y + 1, z)
        elif direction == mazeEnums.absDirection.WEST:
            self.currentPosition = (x - 1, y, z)
        self.updateFrontDirection(direction)

    def updateFrontDirection(self, direction: mazeEnums.absDirection) -> None:
        """
        @brief 前方方向を設定し、Arduino Nano Every の表示を更新する
        @param direction: 設定する方向
        """
        self.frontDirection = direction
        self.updateArduinoStatus()

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
        self.lastCheckpoint = self.currentPosition

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
            self.updateArduinoStatus()

    def _is_known_cell(self, x: int, y: int, z: int) -> bool:
        if self.tileTypes[z][y][x] != mazeEnums.tileType.UNKNOWN:
            return True
        wt = self.wallTypes[z][y][x]
        return any(v != mazeEnums.wallType.UNKNOWN for v in wt.values())

    def _get_known_bounds(self) -> tuple[int, int, int, int, int, int]:
        """known な情報(タイル/壁)が存在する範囲に切り詰めた bbox を返す。

        戻り値: (min_x, min_y, max_x, max_y) いずれも inclusive。
        """
        min_x = self.maxSize
        min_y = self.maxSize
        min_z = self.maxLayer
        max_x = -1
        max_y = -1
        max_z = -1

        for z in range(self.maxLayer):
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
                        if z < min_z:
                            min_z = z
                        if z > max_z:
                            max_z = z
                    

        # 少なくとも START が known のはずだが、念のため。
        if max_x < 0:
            cx, cy, cz = self.currentPosition
            return cx, cy, cx, cy, cz, cz
        return min_x, min_y, max_x, max_y, min_z, max_z

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
        min_x, min_y, max_x, max_y, min_z, max_z = self._get_known_bounds()

        def tile_char(x: int, y: int, z: int) -> str:
            if (x, y, z) == self.currentPosition:
                return "@"
            t = self.tileTypes[y][x]
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
            f"Known map area: x={min_x}..{max_x}, y={min_y}..{max_y}, z={min_z}..{max_z} (current=@ at {cx},{cy},{cz})"
        )

        # 上端(NORTH)
        top = ["+"]
        for x in range(min_x, max_x + 1):
            top.append(
                "---" if wall_at(x, min_y, min_z, mazeEnums.absDirection.NORTH) else "   "
            )
            top.append("+")
        lines.append("".join(top))

        for y in range(min_y, max_y + 1):
            row = []
            # 左端(WEST)
            row.append("|" if wall_at(min_x, y, min_z, mazeEnums.absDirection.WEST) else " ")
            for x in range(min_x, max_x + 1):
                row.append(f" {tile_char(x, y, min_z)} ")
                row.append("|" if wall_at(x, y, min_z, mazeEnums.absDirection.EAST) else " ")
            lines.append("".join(row))

            # 下端(SOUTH)
            sep = ["+"]
            for x in range(min_x, max_x + 1):
                sep.append(
                    "---" if wall_at(x, y, min_z, mazeEnums.absDirection.SOUTH) else "   "
                )
                sep.append("+")
            lines.append("".join(sep))

        return "\n".join(lines)

    def _direction_to_display(self, direction: mazeEnums.absDirection) -> int:
        mapping = {
            mazeEnums.absDirection.NORTH: 0,
            mazeEnums.absDirection.EAST: 1,
            mazeEnums.absDirection.SOUTH: 2,
            mazeEnums.absDirection.WEST: 3,
        }
        return mapping[direction]

    def updateArduinoStatus(self) -> None:
        x, y, z = self.currentPosition
        direction = self._direction_to_display(self.frontDirection)
        self.arduinoNanoEvery.update_oled(x, y, direction)
