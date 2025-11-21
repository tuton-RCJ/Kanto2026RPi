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
    

    def setTileType(self, type: mazeEnums.tileType) -> None:
        x, y = self.currentPosition
        self.tileTypes[y][x] = type

        if type == mazeEnums.tileType.BLACK:
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