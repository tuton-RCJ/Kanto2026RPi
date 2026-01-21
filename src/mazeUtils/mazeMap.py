from . import mazeEnums
from . import mazeConstraints
from .device import deviceEnums
import heapq
import itertools
from typing import Callable
import copy

def _turn_quarters(from_dir: mazeEnums.absDirection, to_dir: mazeEnums.absDirection) -> int:
    """Return minimal number of 90-degree turns needed to rotate from from_dir to to_dir."""
    diff = (to_dir.value - from_dir.value) % 360
    diff = min(diff, (360 - diff) % 360)
    return int(diff // 90)


Position = tuple[int, int, int]


def _step_direction(
    current: tuple[int, int] | Position,
    neighbor: tuple[int, int] | Position,
) -> mazeEnums.absDirection:
    cx, cy = current[0], current[1]
    nx, ny = neighbor[0], neighbor[1]
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
    mazeGraph: dict[int, list[list[set[Position]]]],
    start: Position,
    startDirection: mazeEnums.absDirection,
    goalCondition: Callable[[Position], bool],
) -> list[Position] | None:
    """Rotation-aware Dijkstra.

    State includes heading. Edge cost = (turn quarters * mazeConsrains.TURN_90_SEC) + mazeConsrains.MOVE_STRAIGHT_SEC.
    Returns a path as list of (x, y) positions.
    """

    start_state = (start[0], start[1], start[2], startDirection)
    dist: dict[tuple[int, int, int, mazeEnums.absDirection], float] = {start_state: 0.0}
    prev: dict[tuple[int, int, int, mazeEnums.absDirection], tuple[int, int, int, mazeEnums.absDirection] | None] = {start_state: None}

    # heapq compares tuple elements left-to-right. If earlier fields tie, it would
    # eventually compare absDirection, which isn't orderable -> TypeError.
    # Add a unique tiebreaker to guarantee comparability.
    push_id = itertools.count()
    heap: list[tuple[float, int, int, int, int, mazeEnums.absDirection]] = [
        (0.0, next(push_id), start[0], start[1], start[2], startDirection)
    ]

    while heap:
        cost, _, x, y, level, heading = heapq.heappop(heap)
        state = (x, y, level, heading)
        if cost != dist.get(state):
            continue

        pos = (x, y, level)
        if goalCondition(pos):
            # Reconstruct via state chain, then drop heading.
            states: list[tuple[int, int, int, mazeEnums.absDirection]] = []
            cur: tuple[int, int, int, mazeEnums.absDirection] | None = state
            while cur is not None:
                states.append(cur)
                cur = prev[cur]
            states.reverse()

            path: list[Position] = []
            for sx, sy, sl, _ in states:
                if not path or path[-1] != (sx, sy, sl):
                    path.append((sx, sy, sl))
            return path

        for neighbor in mazeGraph[level][y][x]:
            move_dir = _step_direction(pos, neighbor)
            turn_q = _turn_quarters(heading, move_dir)
            step_cost = (turn_q * float(mazeConstraints.TURN_90_SEC)) + float(mazeConstraints.MOVE_STRAIGHT_SEC)
            new_cost = cost + step_cost
            nx, ny, nl = neighbor
            new_state = (nx, ny, nl, move_dir)

            if new_cost < dist.get(new_state, float("inf")):
                dist[new_state] = new_cost
                prev[new_state] = state
                heapq.heappush(heap, (new_cost, next(push_id), nx, ny, nl, move_dir))

    return None

class mazeMap:
    def __init__(self, maxSize:int = 40):
        self.maxSize = maxSize
        self.tileTypes: dict[int, list[list[mazeEnums.tileType]]] = {}
        self.wallTypes: dict[int, list[list[dict[mazeEnums.absDirection, mazeEnums.wallType]]]] = {}
        self.victimTypes: dict[int, list[list[dict[mazeEnums.absDirection, set[deviceEnums.UnitVStatus]]]]] = {}
        self.wallSeenCount: dict[int, list[list[dict[mazeEnums.absDirection, int]]]] = {}
        self.mazeAsGraph: dict[int, list[list[set[Position]]]] = {}
        self.rampLevelLinks: dict[int, tuple[int, int | None]] = {}
        self._rampLevelCounter: int = mazeConstraints.RAMP_LEVEL_BASE
        self.renderLevelOverride: int | None = None

        self._ensure_level(mazeConstraints.START_LEVEL)
        self.currentPosition = (maxSize // 2, maxSize // 2, mazeConstraints.START_LEVEL)
        self.tileTypes[mazeConstraints.START_LEVEL][maxSize // 2][maxSize // 2] = mazeEnums.tileType.START
        self.frontDirection = mazeEnums.absDirection.NORTH
        self.nowRescueKitCount = mazeConstraints.DEFAULT_RESCUE_KIT_COUNT.copy()
        self.savedCache = dict()
        self.lastCheckpoint = self.currentPosition
        self._lastMoveDirection: mazeEnums.absDirection | None = None
        self._lastMoveLevelDelta: int = 0
        self.saveCache()

    def _ensure_level(self, level: int) -> None:
        if level in self.tileTypes:
            return
        self.wallTypes[level] = [[{d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection} for _ in range(self.maxSize)] for _ in range(self.maxSize)]
        self.tileTypes[level] = [[mazeEnums.tileType.UNKNOWN for _ in range(self.maxSize)] for _ in range(self.maxSize)]
        self.victimTypes[level] = [[{d: set() for d in mazeEnums.absDirection} for _ in range(self.maxSize)] for _ in range(self.maxSize)]
        self.wallSeenCount[level] = [[{d: 0 for d in mazeEnums.absDirection} for _ in range(self.maxSize)] for _ in range(self.maxSize)]
        self.mazeAsGraph[level] = [[set() for _ in range(self.maxSize)] for _ in range(self.maxSize)]

    def createRampLevel(self, entryLevel: int) -> int:
        rampLevel = self._rampLevelCounter
        self._rampLevelCounter += 1
        self.rampLevelLinks[rampLevel] = (entryLevel, None)
        self._ensure_level(rampLevel)
        return rampLevel

    def finalizeRampLevel(self, rampLevel: int, exitLevel: int) -> None:
        if rampLevel in self.rampLevelLinks:
            entryLevel, _ = self.rampLevelLinks[rampLevel]
            self.rampLevelLinks[rampLevel] = (entryLevel, exitLevel)

    def setWallType(self, direction: mazeEnums.absDirection, wallType: mazeEnums.wallType) -> None:
        x, y, level = self.currentPosition
        self._ensure_level(level)

        neighbor_level = level
        if self._lastMoveLevelDelta != 0 and self._lastMoveDirection is not None:
            if direction == self._lastMoveDirection.opposite():
                neighbor_level = level - self._lastMoveLevelDelta
        self._ensure_level(neighbor_level)

        # もし壁がないならグラフを更新
        if wallType == mazeEnums.wallType.NO_WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                if self.tileTypes[neighbor_level][y-1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[level][y][x].add((x, y-1, neighbor_level))
                    self.mazeAsGraph[neighbor_level][y-1][x].add((x, y, level))
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                if self.tileTypes[neighbor_level][y][x+1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[level][y][x].add((x+1, y, neighbor_level))
                    self.mazeAsGraph[neighbor_level][y][x+1].add((x, y, level))
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                if self.tileTypes[neighbor_level][y+1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[level][y][x].add((x, y+1, neighbor_level))
                    self.mazeAsGraph[neighbor_level][y+1][x].add((x, y, level))
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                if self.tileTypes[neighbor_level][y][x-1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[level][y][x].add((x-1, y, neighbor_level))
                    self.mazeAsGraph[neighbor_level][y][x-1].add((x, y, level))

        self.wallTypes[level][y][x][direction] = wallType

    def getWallType(self, direction: mazeEnums.absDirection = None) -> dict[mazeEnums.absDirection, mazeEnums.wallType]:
        """
        @brief 現在位置の壁タイプを取得する
        @return: 現在位置の壁タイプの辞書
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        if direction is not None:
            if direction == mazeEnums.absDirection.NORTH:
                y = y - 1
                return self.wallTypes[level][y][x]
            elif direction == mazeEnums.absDirection.EAST:
                x = x + 1
                return self.wallTypes[level][y][x]
            elif direction == mazeEnums.absDirection.SOUTH:
                y = y + 1
                return self.wallTypes[level][y][x]
            elif direction == mazeEnums.absDirection.WEST:
                x = x - 1
                return self.wallTypes[level][y][x]
        return self.wallTypes[level][y][x]
    
    def getVictimTypes(self, direction: mazeEnums.absDirection = None) -> dict[mazeEnums.absDirection, deviceEnums.UnitVStatus]:
        """
        @brief 現在位置の被災者タイプを取得する
        @return: 現在位置の被災者タイプの辞書
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        if direction is not None:
            if direction == mazeEnums.absDirection.NORTH:
                y -= 1
            elif direction == mazeEnums.absDirection.EAST:
                x += 1
            elif direction == mazeEnums.absDirection.SOUTH:
                y += 1
            elif direction == mazeEnums.absDirection.WEST:
                x -= 1
            return copy.deepcopy(self.victimTypes[level][y][x])
        return copy.deepcopy(self.victimTypes[level][y][x])
    
    def getSeenCount(self) -> dict[mazeEnums.absDirection, int]:
        """
        @brief 現在位置の壁の検出回数を取得する
        @return: 現在位置の壁の検出回数の辞書
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        return self.wallSeenCount[level][y][x]

    def getTileType(self) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        return self.tileTypes[level][y][x]
    
    def addVictimType(self, victimType: deviceEnums.UnitVStatus, direction: mazeEnums.absDirection, tileDirection: mazeEnums.absDirection = None) -> None:
        """
        @brief: 指定した方向の壁に被災者タイプを追加する。tileDirection が None の場合は現在地に、そうでない場合はその方向のタイルの壁に追加する
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        if not tileDirection is None:
            if tileDirection == mazeEnums.absDirection.NORTH:
                y -= 1
            elif tileDirection == mazeEnums.absDirection.EAST:
                x += 1
            elif tileDirection == mazeEnums.absDirection.SOUTH:
                y += 1
            elif tileDirection == mazeEnums.absDirection.WEST:
                x -= 1
    
        self.victimTypes[level][y][x][direction].add(victimType)

    def setTileType(self, tiletype: mazeEnums.tileType, direction: mazeEnums.absDirection = None) -> None:
        """
        @brief 指定した方向のタイルタイプを設定する。directionがNoneの場合は現在位置に設定する
        @param tiletype: 設定するタイルタイプ
        @param direction: 設定する方向
        """
        if direction is None:
            x, y, level = self.currentPosition
        else:
            x, y, level = self.currentPosition
            if direction == mazeEnums.absDirection.NORTH:
                y -= 1
            elif direction == mazeEnums.absDirection.EAST:
                x += 1
            elif direction == mazeEnums.absDirection.SOUTH:
                y += 1
            elif direction == mazeEnums.absDirection.WEST:
                x -= 1
        self._ensure_level(level)
        assert 0 <= x < self.maxSize and 0 <= y < self.maxSize, self.renderKnownTileAndWall()
        self.tileTypes[level][y][x] = tiletype

        if tiletype == mazeEnums.tileType.BLACK:
            # 黒タイルならその周囲の通路を塞ぐ(同一/異なるレベルも含む)
            for neighbor in list(self.mazeAsGraph[level][y][x]):
                nx, ny, nl = neighbor
                self.mazeAsGraph[level][y][x].discard(neighbor)
                self.mazeAsGraph[nl][ny][nx].discard((x, y, level))

        if tiletype == mazeEnums.tileType.SILVER:
            self.saveCache()
            

    

    def getAroundTileType(self) -> dict[mazeEnums.absDirection, mazeEnums.tileType]:
        """
        @brief 現在位置の周囲のタイルタイプを取得する
        @return: 周囲のタイルタイプの辞書
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        aroundTiles = {d: mazeEnums.tileType.UNKNOWN for d in mazeEnums.absDirection}
        
        if y > 0:
            aroundTiles[mazeEnums.absDirection.NORTH] = self.tileTypes[level][y-1][x]
        if x < self.maxSize - 1:
            aroundTiles[mazeEnums.absDirection.EAST] = self.tileTypes[level][y][x+1]
        if y < self.maxSize - 1:
            aroundTiles[mazeEnums.absDirection.SOUTH] = self.tileTypes[level][y+1][x]
        if x > 0:
            aroundTiles[mazeEnums.absDirection.WEST] = self.tileTypes[level][y][x-1]
        
        return aroundTiles


    def getCurrentTileType(self) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        return self.tileTypes[level][y][x]


    def getNearestUnexploredTile(self) -> list[mazeEnums.absDirection] | None:
        """
        @brief 最も近い未探索タイルへのパスを取得する
        @return: 未探索タイルへの方向リスト。未探索タイルが存在しない場合は None を返す
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        path = dijkstra(
            self.mazeAsGraph,
            (x, y, level),
            self.frontDirection,
            lambda pos: pos != (x, y, level)
            and self.tileTypes[pos[2]][pos[1]][pos[0]] == mazeEnums.tileType.UNKNOWN,
        )

        if path is None:
            return None
        
        directions = []
        
        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, _ = path[i-1]
            nextX, nextY, _ = path[i]
            if nextX == currX and nextY == currY - 1:
                directions.append(mazeEnums.absDirection.NORTH)
            elif nextX == currX + 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.EAST)
            elif nextX == currX and nextY == currY + 1:
                directions.append(mazeEnums.absDirection.SOUTH)
            elif nextX == currX - 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.WEST)
        
        return directions
    
    def getPathTo(self, target: tuple[int, int] | Position) -> list[mazeEnums.absDirection] | None:
        """
        @brief 指定した座標へのパスを取得する
        @param target: 目的地の座標 (x, y) or (x, y, level)
        @return: 目的地への方向リスト。到達不可能な場合は None を返す
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        if len(target) == 2:
            target_pos: Position = (target[0], target[1], level)
        else:
            target_pos = (target[0], target[1], target[2])
        self._ensure_level(target_pos[2])

        path = dijkstra(self.mazeAsGraph, (x, y, level), self.frontDirection, lambda pos: pos == target_pos)

        if path is None:
            return None
        
        directions = []
        
        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY, _ = path[i-1]
            nextX, nextY, _ = path[i]
            if nextX == currX and nextY == currY - 1:
                directions.append(mazeEnums.absDirection.NORTH)
            elif nextX == currX + 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.EAST)
            elif nextX == currX and nextY == currY + 1:
                directions.append(mazeEnums.absDirection.SOUTH)
            elif nextX == currX - 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.WEST)
        
        return directions
    
    def moveTo(self, direction: mazeEnums.absDirection, levelDelta: int = 0) -> None:
        """
        @brief 指定した方向に移動する
        @param direction: 移動する方向
        @param levelDelta: レベル差分（上り:+1/下り:-1）
        """
        x, y, level = self.currentPosition
        self._ensure_level(level)
        if self.wallTypes[level][y][x][direction] != mazeEnums.wallType.NO_WALL:
            print(self.renderKnownTileAndWall())

        if direction == mazeEnums.absDirection.NORTH:
            self.currentPosition = (x, y-1, level + levelDelta)
        elif direction == mazeEnums.absDirection.EAST:
            self.currentPosition = (x+1, y, level + levelDelta)
        elif direction == mazeEnums.absDirection.SOUTH:
            self.currentPosition = (x, y+1, level + levelDelta)
        elif direction == mazeEnums.absDirection.WEST:
            self.currentPosition = (x-1, y, level + levelDelta)
        self.frontDirection = direction
        self._ensure_level(self.currentPosition[2])
        self._lastMoveDirection = direction
        self._lastMoveLevelDelta = levelDelta
        
    def setFrontDirection(self, direction: mazeEnums.absDirection) -> None:
        """
        @brief 前方方向を設定する
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
        self.savedCache['tileTypes'] = {level: [row.copy() for row in grid] for level, grid in self.tileTypes.items()}
        self.savedCache['wallTypes'] = {level: [[{d: wt[d] for d in mazeEnums.absDirection} for wt in row] for row in grid] for level, grid in self.wallTypes.items()}
        self.savedCache['mazeAsGraph'] = {level: [[neighbors.copy() for neighbors in row] for row in grid] for level, grid in self.mazeAsGraph.items()}
        self.savedCache['rampLevelLinks'] = {k: (v[0], v[1]) for k, v in self.rampLevelLinks.items()}
        self.savedCache['rampLevelCounter'] = self._rampLevelCounter
        self.lastCheckpoint = self.currentPosition

    def loadCache(self, nowDirection: mazeEnums.absDirection) -> None:
        """
        @brief キャッシュからマップ状態を復元する
        @param nowDirection: 現在の前方方向
        """
        if 'tileTypes' in self.savedCache and 'wallTypes' in self.savedCache:
            self.tileTypes = {level: [row.copy() for row in grid] for level, grid in self.savedCache['tileTypes'].items()}
            self.wallTypes = {level: [[{d: wt[d] for d in mazeEnums.absDirection} for wt in row] for row in grid] for level, grid in self.savedCache['wallTypes'].items()}
            self.mazeAsGraph = {level: [[neighbors.copy() for neighbors in row] for row in grid] for level, grid in self.savedCache['mazeAsGraph'].items()}
            self.rampLevelLinks = {k: (v[0], v[1]) for k, v in self.savedCache.get('rampLevelLinks', {}).items()}
            self._rampLevelCounter = int(self.savedCache.get('rampLevelCounter', self._rampLevelCounter))
            self.frontDirection = nowDirection
            self.currentPosition = self.lastCheckpoint
            self._ensure_level(self.currentPosition[2])
            self._lastMoveDirection = None
            self._lastMoveLevelDelta = 0
            
    def _is_known_cell(self, level: int, x: int, y: int) -> bool:
        self._ensure_level(level)
        if self.tileTypes[level][y][x] != mazeEnums.tileType.UNKNOWN:
            return True
        wt = self.wallTypes[level][y][x]
        return any(v != mazeEnums.wallType.UNKNOWN for v in wt.values())

    def _get_known_bounds(self, level: int) -> tuple[int, int, int, int]:
        """known な情報(タイル/壁)が存在する範囲に切り詰めた bbox を返す。

        戻り値: (min_x, min_y, max_x, max_y) いずれも inclusive。
        """
        min_x = self.maxSize
        min_y = self.maxSize
        max_x = -1
        max_y = -1

        for y in range(self.maxSize):
            for x in range(self.maxSize):
                if self._is_known_cell(level, x, y):
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
            cx, cy, _ = self.currentPosition
            return cx, cy, cx, cy
        return min_x, min_y, max_x, max_y

    def setRenderLevel(self, level: int | None) -> None:
        """
        @brief renderKnownTileAndWall の表示レベルを固定する。Noneで現在レベルに戻す。
        """
        self.renderLevelOverride = level

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

    def renderKnownTileAndWall(self, level: int | None = None) -> str:
        """tileTypes と wallTypes をまとめて、UNKNOWNのみの行列が出ない範囲でASCII表示する。"""
        if level is None:
            level = self.renderLevelOverride if self.renderLevelOverride is not None else self.currentPosition[2]
        if level not in self.tileTypes:
            return f"Level {level} does not exist."
        min_x, min_y, max_x, max_y = self._get_known_bounds(level)

        def tile_char(x: int, y: int) -> str:
            cx, cy, cz = self.currentPosition
            if (x, y) == (cx, cy) and level == cz:
                return "@"
            t = self.tileTypes[level][y][x]
            # tileType は __str__ 実装済み(U/E/R/...)。
            return str(t)

        def wall_at(x: int, y: int, d: mazeEnums.absDirection) -> bool:
            """(x,y) の d 方向の壁を、両側タイルの情報で OR 判定して返す。

            片側だけ更新されて不整合が起きても、「どちらかが壁」なら壁として描画する。
            """
            w1 = self.wallTypes[level][y][x][d]

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

            w2 = self.wallTypes[level][ny][nx][od]
            b2 = self._wall_as_bool(w2)
            return b1 or b2

        lines: list[str] = []
        cx, cy, cz = self.currentPosition
        lines.append(f"Known map area: x={min_x}..{max_x}, y={min_y}..{max_y}, level={level} (current=@ at {cx},{cy},{cz})")

        # 上端(NORTH)
        top = ["+"]
        for x in range(min_x, max_x + 1):
            top.append("---" if wall_at(x, min_y, mazeEnums.absDirection.NORTH) else "   ")
            top.append("+")
        lines.append("".join(top))

        for y in range(min_y, max_y + 1):
            row = []
            # 左端(WEST)
            row.append("|" if wall_at(min_x, y, mazeEnums.absDirection.WEST) else " ")
            for x in range(min_x, max_x + 1):
                row.append(f" {tile_char(x, y)} ")
                row.append("|" if wall_at(x, y, mazeEnums.absDirection.EAST) else " ")
            lines.append("".join(row))

            # 下端(SOUTH)
            sep = ["+"]
            for x in range(min_x, max_x + 1):
                sep.append("---" if wall_at(x, y, mazeEnums.absDirection.SOUTH) else "   ")
                sep.append("+")
            lines.append("".join(sep))

        return "\n".join(lines)