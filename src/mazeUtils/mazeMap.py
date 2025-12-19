from . import mazeEnums
from collections import deque

def BFS(mazeGraph: list[list[set]], start: tuple[int, int], goalCondition) -> list[tuple[int, int]] | None:

    queue = deque([start])
    visited = {start: None}

    while queue:
        current = queue.popleft()
        # print(current, visited)
        if goalCondition(current):
            path = []
            while current is not None:
                path.append(current)
                current = visited[current]
            return path[::-1]

        # mazeGraph is accessed as [y][x], current is (x, y)
        for neighbor in mazeGraph[current[1]][current[0]]:
            if neighbor not in visited:
                visited[neighbor] = current
                queue.append(neighbor)

    return None

# 初期位置は必ずロボットの前方が開くように設定。
class mazeMap:
    def __init__(self, maxSize:int = 40, loadCache:bool = False, cacheAbsPath:str = "mazeCache.json"):
        self.maxSize = maxSize
        self.cacheAbsPath = cacheAbsPath
        self.loadCache = loadCache
        self.currentPosition = (maxSize // 2, maxSize // 2)
        self.wallTypes = [[{d: mazeEnums.wallType.UNKNOWN for d in mazeEnums.absDirection} for _ in range(maxSize)] for _ in range(maxSize)]
        self.tileTypes = [[mazeEnums.tileType.UNKNOWN for _ in range(maxSize)] for _ in range(maxSize)]
        self.tileTypes[maxSize // 2][maxSize // 2] = mazeEnums.tileType.START
        self.mazeAsGraph = [[set() for _ in range(maxSize)] for _ in range(maxSize)]
        self.frontDirection = mazeEnums.absDirection.NORTH
        if loadCache: #TODO: cache の実装
            pass 

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


    def getWallType(self) -> dict[mazeEnums.absDirection, mazeEnums.wallType]:
        x, y = self.currentPosition
        return self.wallTypes[y][x]
    

    def setTileType(self, tiletype: mazeEnums.tileType, direction: mazeEnums.absDirection = None) -> None:
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
        self.tileTypes[y][x] = tiletype

        if tiletype == mazeEnums.tileType.BLACK:
            # 黒タイルならその周囲の通路を塞ぐ
            for direction in mazeEnums.absDirection:
                self.wallTypes[y][x][direction] = mazeEnums.wallType.WALL
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

    

    def getAroundTileType(self) -> dict[mazeEnums.absDirection, mazeEnums.tileType]:
        
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
        x, y = self.currentPosition
        return self.tileTypes[y][x]


    def getNearestUnexploredTile(self) -> list[mazeEnums.absDirection]:
        """
        @brief 最も近い未探索タイルへのパスを取得する
        @return: 未探索タイルへの方向リスト。未探索タイルが存在しない場合は None を返す
        """
        x, y = self.currentPosition
        path = BFS(self.mazeAsGraph, (x, y), lambda pos: any(self.tileTypes[pos[1]][pos[0]] == mazeEnums.tileType.UNKNOWN for d in mazeEnums.absDirection))

        if path is None:
            print("None!!!!!!")
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
        path = BFS(self.mazeAsGraph, (x, y), lambda pos: pos == target)

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
        x, y = self.currentPosition
        assert self.wallTypes[y][x][direction] == mazeEnums.wallType.NO_WALL, "Cannot move in the specified direction; wall is present."

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
        self.frontDirection = direction

    def saveCache(self) -> None: #TODO: cache の実装
        pass

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
            t = self.tileTypes[y][x]
            # tileType は __str__ 実装済み(U/E/R/...)。
            return str(t)

        def wall_at(x: int, y: int, d: mazeEnums.absDirection) -> bool:
            return self._wall_as_bool(self.wallTypes[y][x][d])

        lines: list[str] = []
        lines.append(f"Known map area: x={min_x}..{max_x}, y={min_y}..{max_y}")

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