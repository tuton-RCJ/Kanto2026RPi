from . import mazeEnums
from . import mazeConstraints
from .device import deviceEnums
import heapq
import itertools
from typing import Callable

def _turn_quarters(from_dir: mazeEnums.absDirection, to_dir: mazeEnums.absDirection) -> int:
    """Return minimal number of 90-degree turns needed to rotate from from_dir to to_dir."""
    diff = (to_dir.value - from_dir.value) % 360
    diff = min(diff, (360 - diff) % 360)
    return int(diff // 90)


def _step_direction(
    current: tuple[int, int],
    neighbor: tuple[int, int],
) -> mazeEnums.absDirection:
    cx, cy = current
    nx, ny = neighbor
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
    mazeGraph: list[list[set[tuple[int, int]]]],
    start: tuple[int, int],
    startDirection: mazeEnums.absDirection,
    goalCondition: Callable[[tuple[int, int]], bool],
) -> list[tuple[int, int]] | None:
    """Rotation-aware Dijkstra.

    State includes heading. Edge cost = (turn quarters * mazeConsrains.TURN_90_SEC) + mazeConsrains.MOVE_STRAIGHT_SEC.
    Returns a path as list of (x, y) positions.
    """

    start_state = (start[0], start[1], startDirection)
    dist: dict[tuple[int, int, mazeEnums.absDirection], float] = {start_state: 0.0}
    prev: dict[tuple[int, int, mazeEnums.absDirection], tuple[int, int, mazeEnums.absDirection] | None] = {start_state: None}

    # heapq compares tuple elements left-to-right. If earlier fields tie, it would
    # eventually compare absDirection, which isn't orderable -> TypeError.
    # Add a unique tiebreaker to guarantee comparability.
    push_id = itertools.count()
    heap: list[tuple[float, int, int, int, mazeEnums.absDirection]] = [
        (0.0, next(push_id), start[0], start[1], startDirection)
    ]

    while heap:
        cost, _, x, y, heading = heapq.heappop(heap)
        state = (x, y, heading)
        if cost != dist.get(state):
            continue

        pos = (x, y)
        if goalCondition(pos):
            # Reconstruct via state chain, then drop heading.
            states: list[tuple[int, int, mazeEnums.absDirection]] = []
            cur: tuple[int, int, mazeEnums.absDirection] | None = state
            while cur is not None:
                states.append(cur)
                cur = prev[cur]
            states.reverse()

            path: list[tuple[int, int]] = []
            for sx, sy, _ in states:
                if not path or path[-1] != (sx, sy):
                    path.append((sx, sy))
            return path

        for neighbor in mazeGraph[y][x]:
            move_dir = _step_direction(pos, neighbor)
            turn_q = _turn_quarters(heading, move_dir)
            step_cost = (turn_q * float(mazeConstraints.TURN_90_SEC)) + float(mazeConstraints.MOVE_STRAIGHT_SEC)
            new_cost = cost + step_cost
            nx, ny = neighbor
            new_state = (nx, ny, move_dir)

            if new_cost < dist.get(new_state, float("inf")):
                dist[new_state] = new_cost
                prev[new_state] = state
                heapq.heappush(heap, (new_cost, next(push_id), nx, ny, move_dir))

    return None

class mazeMap:
    def __init__(self, maxSize:int = 40):
        self.maxSize = maxSize
        self.currentPosition = (maxSize // 2, maxSize // 2)
        self.wallTypes = [[{d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection} for _ in range(maxSize)] for _ in range(maxSize)]
        self.tileTypes = [[mazeEnums.tileType.UNKNOWN for _ in range(maxSize)] for _ in range(maxSize)]
        self.wallTypesOnly45Deg = [[{d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection} for _ in range(maxSize)] for _ in range(maxSize)] 
        self.wallSeenCount = [[{d: 0 for d in mazeEnums.absDirection} for _ in range(maxSize)] for _ in range(maxSize)]
        self.tileTypes[maxSize // 2][maxSize // 2] = mazeEnums.tileType.START
        self.mazeAsGraph = [[set() for _ in range(maxSize)] for _ in range(maxSize)]
        self.frontDirection = mazeEnums.absDirection.NORTH
        self.nowRescueKitCount = mazeConstraints.DEFAULT_RESCUE_KIT_COUNT.copy()
        self.savedCache = dict()
        self.lastCheckpoint = self.currentPosition
        self.saveCache()

    def setWallType(self, direction: mazeEnums.absDirection, wallType: mazeEnums.wallType) -> None:
        x, y = self.currentPosition

        # もし壁がないならグラフを更新
        if wallType == mazeEnums.wallType.NO_WALL:
            if direction == mazeEnums.absDirection.NORTH and y > 0:
                if self.tileTypes[y-1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[y][x].add((x, y-1))
                    self.mazeAsGraph[y-1][x].add((x, y))
            elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                if self.tileTypes[y][x+1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[y][x].add((x+1, y))
                    self.mazeAsGraph[y][x+1].add((x, y))
            elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                if self.tileTypes[y+1][x] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[y][x].add((x, y+1))
                    self.mazeAsGraph[y+1][x].add((x, y))
            elif direction == mazeEnums.absDirection.WEST and x > 0:
                if self.tileTypes[y][x-1] != mazeEnums.tileType.BLACK:
                    self.mazeAsGraph[y][x].add((x-1, y))
                    self.mazeAsGraph[y][x-1].add((x, y))

        self.wallTypes[y][x][direction] = wallType

    def getWallType(self, direction: mazeEnums.absDirection = None) -> dict[mazeEnums.absDirection, mazeEnums.wallType]:
        """
        @brief 現在位置の壁タイプを取得する
        @return: 現在位置の壁タイプの辞書
        """
        if direction is not None:
            if direction == mazeEnums.absDirection.NORTH:
                y = self.currentPosition[1] - 1
                return self.wallTypes[y][self.currentPosition[0]]
            elif direction == mazeEnums.absDirection.EAST:
                x = self.currentPosition[0] + 1
                return self.wallTypes[self.currentPosition[1]][x]
            elif direction == mazeEnums.absDirection.SOUTH:
                y = self.currentPosition[1] + 1
                return self.wallTypes[y][self.currentPosition[0]]
            elif direction == mazeEnums.absDirection.WEST:
                x = self.currentPosition[0] - 1
                return self.wallTypes[self.currentPosition[1]][x]
        x, y = self.currentPosition
        return self.wallTypes[y][x]
    
    def getSeenCount(self) -> dict[mazeEnums.absDirection, int]:
        """
        @brief 現在位置の壁の検出回数を取得する
        @return: 現在位置の壁の検出回数の辞書
        """
        x, y = self.currentPosition
        return self.wallSeenCount[y][x]

    def getTileType(self) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y = self.currentPosition
        return self.tileTypes[y][x]
        
    def addSeenCount(self) -> None:
        """
        @brief: nowDirection に対して水平な壁の検出回数を増やす
        """
        x, y = self.currentPosition
        for direction in mazeEnums.absDirection:
            if direction == mazeEnums.absDirection((self.frontDirection.value + 90) % 360) or direction == mazeEnums.absDirection((self.frontDirection.value + 270) % 360):
                self.wallSeenCount[y][x][direction] += 1
    def setTileType(self, tiletype: mazeEnums.tileType, direction: mazeEnums.absDirection = None) -> None:
        """
        @brief 指定した方向のタイルタイプを設定する。directionがNoneの場合は現在位置に設定する
        @param tiletype: 設定するタイルタイプ
        @param direction: 設定する方向
        """
        if direction is None:
            x, y = self.currentPosition
        else:
            x, y = self.currentPosition
            if direction == mazeEnums.absDirection.NORTH:
                y -= 1
            elif direction == mazeEnums.absDirection.EAST:
                x += 1
            elif direction == mazeEnums.absDirection.SOUTH:
                y += 1
            elif direction == mazeEnums.absDirection.WEST:
                x -= 1
        assert 0 <= x < self.maxSize and 0 <= y < self.maxSize, self.renderKnownTileAndWall()
        self.tileTypes[y][x] = tiletype

        if tiletype == mazeEnums.tileType.BLACK:
            # 黒タイルならその周囲の通路を塞ぐ
            for direction in mazeEnums.absDirection:
                if direction == mazeEnums.absDirection.NORTH and y > 0:
                    self.mazeAsGraph[y][x].discard((x, y-1))
                    self.mazeAsGraph[y-1][x].discard((x, y))
                elif direction == mazeEnums.absDirection.EAST and x < self.maxSize - 1:
                    self.mazeAsGraph[y][x].discard((x+1, y))
                    self.mazeAsGraph[y][x+1].discard((x, y))
                elif direction == mazeEnums.absDirection.SOUTH and y < self.maxSize - 1:
                    self.mazeAsGraph[y][x].discard((x, y+1))
                    self.mazeAsGraph[y+1][x].discard((x, y))
                elif direction == mazeEnums.absDirection.WEST and x > 0:
                    self.mazeAsGraph[y][x].discard((x-1, y))
                    self.mazeAsGraph[y][x-1].discard((x, y))

        if tiletype == mazeEnums.tileType.SILVER:
            self.saveCache()
            

    

    def getAroundTileType(self) -> dict[mazeEnums.absDirection, mazeEnums.tileType]:
        """
        @brief 現在位置の周囲のタイルタイプを取得する
        @return: 周囲のタイルタイプの辞書
        """
        x, y = self.currentPosition
        aroundTiles = {d: mazeEnums.tileType.UNKNOWN for d in mazeEnums.absDirection}
        
        if y > 0:
            aroundTiles[mazeEnums.absDirection.NORTH] = self.tileTypes[y-1][x]
        if x < self.maxSize - 1:
            aroundTiles[mazeEnums.absDirection.EAST] = self.tileTypes[y][x+1]
        if y < self.maxSize - 1:
            aroundTiles[mazeEnums.absDirection.SOUTH] = self.tileTypes[y+1][x]
        if x > 0:
            aroundTiles[mazeEnums.absDirection.WEST] = self.tileTypes[y][x-1]
        
        return aroundTiles


    def getCurrentTileType(self) -> mazeEnums.tileType:
        """
        @brief 現在位置のタイルタイプを取得する
        @return: 現在位置のタイルタイプ
        """
        x, y = self.currentPosition
        return self.tileTypes[y][x]


    def getNearestUnexploredTile(self) -> list[mazeEnums.absDirection] | None:
        """
        @brief 最も近い未探索タイルへのパスを取得する
        @return: 未探索タイルへの方向リスト。未探索タイルが存在しない場合は None を返す
        """
        x, y = self.currentPosition
        path = dijkstra(
            self.mazeAsGraph,
            (x, y),
            self.frontDirection,
            lambda pos: pos != (x, y)
            and self.tileTypes[pos[1]][pos[0]] == mazeEnums.tileType.UNKNOWN,
        )

        if path is None:
            return None
        
        directions = []
        
        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY = path[i-1]
            nextX, nextY = path[i]
            if nextX == currX and nextY == currY - 1:
                directions.append(mazeEnums.absDirection.NORTH)
            elif nextX == currX + 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.EAST)
            elif nextX == currX and nextY == currY + 1:
                directions.append(mazeEnums.absDirection.SOUTH)
            elif nextX == currX - 1 and nextY == currY:
                directions.append(mazeEnums.absDirection.WEST)
        
        return directions
    
    def getPathTo(self, target: tuple[int, int]) -> list[mazeEnums.absDirection] | None:
        """
        @brief 指定した座標へのパスを取得する
        @param target: 目的地の座標 (x, y)
        @return: 目的地への方向リスト。到達不可能な場合は None を返す
        """
        x, y = self.currentPosition
        path = dijkstra(self.mazeAsGraph, (x, y), self.frontDirection, lambda pos: pos == target)

        if path is None:
            return None
        
        directions = []
        
        # パスの各ステップを方向に変換
        for i in range(1, len(path)):
            currX, currY = path[i-1]
            nextX, nextY = path[i]
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
        x, y = self.currentPosition
        if self.wallTypes[y][x][direction] != mazeEnums.wallType.NO_WALL:
            print(self.renderKnownTileAndWall())

        if direction == mazeEnums.absDirection.NORTH:
            self.currentPosition = (x, y-1)
        elif direction == mazeEnums.absDirection.EAST:
            self.currentPosition = (x+1, y)
        elif direction == mazeEnums.absDirection.SOUTH:
            self.currentPosition = (x, y+1)
        elif direction == mazeEnums.absDirection.WEST:
            self.currentPosition = (x-1, y)
        self.frontDirection = direction
        
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
        self.savedCache['tileTypes'] = [row.copy() for row in self.tileTypes]
        self.savedCache['wallTypes'] = [[{d: wt[d] for d in mazeEnums.absDirection} for wt in row] for row in self.wallTypes]
        self.savedCache['mazeAsGraph'] = [[neighbors.copy() for neighbors in row] for row in self.mazeAsGraph]
        self.lastCheckpoint = self.currentPosition

    def loadCache(self, nowDirection: mazeEnums.absDirection) -> None:
        """
        @brief キャッシュからマップ状態を復元する
        @param nowDirection: 現在の前方方向
        """
        if 'tileTypes' in self.savedCache and 'wallTypes' in self.savedCache:
            self.tileTypes = [row.copy() for row in self.savedCache['tileTypes']]
            self.wallTypes = [[{d: wt[d] for d in mazeEnums.absDirection} for wt in row] for row in self.savedCache['wallTypes']]
            self.mazeAsGraph = [[neighbors.copy() for neighbors in row] for row in self.savedCache['mazeAsGraph']]
            self.frontDirection = nowDirection
            self.currentPosition = self.lastCheckpoint
            
    def _is_known_cell(self, x: int, y: int) -> bool:
        if self.tileTypes[y][x] != mazeEnums.tileType.UNKNOWN:
            return True
        wt = self.wallTypes[y][x]
        return any(v != mazeEnums.wallType.UNKNOWN for v in wt.values())

    def _get_known_bounds(self) -> tuple[int, int, int, int]:
        """known な情報(タイル/壁)が存在する範囲に切り詰めた bbox を返す。

        戻り値: (min_x, min_y, max_x, max_y) いずれも inclusive。
        """
        min_x = self.maxSize
        min_y = self.maxSize
        max_x = -1
        max_y = -1

        for y in range(self.maxSize):
            for x in range(self.maxSize):
                if self._is_known_cell(x, y):
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
            cx, cy = self.currentPosition
            return cx, cy, cx, cy
        return min_x, min_y, max_x, max_y

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
        min_x, min_y, max_x, max_y = self._get_known_bounds()

        def tile_char(x: int, y: int) -> str:
            if (x, y) == self.currentPosition:
                return "@"
            t = self.tileTypes[y][x]
            # tileType は __str__ 実装済み(U/E/R/...)。
            return str(t)

        def wall_at(x: int, y: int, d: mazeEnums.absDirection) -> bool:
            """(x,y) の d 方向の壁を、両側タイルの情報で OR 判定して返す。

            片側だけ更新されて不整合が起きても、「どちらかが壁」なら壁として描画する。
            """
            w1 = self.wallTypes[y][x][d]

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

            w2 = self.wallTypes[ny][nx][od]
            b2 = self._wall_as_bool(w2)
            return b1 or b2

        lines: list[str] = []
        cx, cy = self.currentPosition
        lines.append(f"Known map area: x={min_x}..{max_x}, y={min_y}..{max_y} (current=@ at {cx},{cy})")

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