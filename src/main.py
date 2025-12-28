from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    mapInstance = mazeMap.mazeMap()
    lidarInstance = LiDAR.initializeLidar()
    try:
        while True:
            """
            stmInstance.update()
            print(f"tofDistance: {stmInstance.tof.getDistance()} cm")
            #print(f"AbsAngle: {stmInstance.gyro.getValue().heading}")
            """
            stmInstance.update()
            while stmInstance.switch.getToggleSwitch1():
                stmInstance.update()
            moveTile.detectWall(lidarInstance, mapInstance)
            tileType = moveTile.detectTileColor()
            mapInstance.setTileType(tileType)
            moveTile.rescueVictim(mapInstance, stmInstance)

            print("Initial Map:")
            print(mapInstance.renderKnownTileAndWall())

            nextDirection = mapInstance.getNearestUnexploredTile()
            print(f"Next Direction: {nextDirection}")
            

            print("Exploration started.")
            while nextDirection is not None:
                for direction in nextDirection:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    print(mapInstance.renderKnownTileAndWall())
                    toggleswitchFlag = False
                    stmInstance.update()

                    if stmInstance.switch.getToggleSwitch1():
                        print("Exploration paused. Toggle switch 1 to resume.")

                    while stmInstance.switch.getToggleSwitch1():
                        stmInstance.update()
                        toggleswitchFlag = True
                    
                    if toggleswitchFlag:
                        print("Exploration resumed.")
                        toggleswitchFlag = False
                        nowAngle = stmInstance.gyro.getValue().heading
                        nowDirection = None
                        error = 1e9
                        for direction in mazeEnums.absDirection:
                            diff = abs(nowAngle - direction.value)
                            if diff > 180:
                                diff = 360 - diff
                            if diff < error:
                                error = diff
                                nowDirection = direction
                        mapInstance.loadCache(nowDirection=nowDirection)
                        mapInstance.renderKnownTileAndWall()
                        time.sleep(1)  # Allow time for stabilization after resuming
                        break

                nextDirection = mapInstance.getNearestUnexploredTile()
                    
            returnPath = mapInstance.getPathTo((20, 20))
            print(f"Return Path: {returnPath}")
            
            if returnPath:
                for direction in returnPath:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    print(mapInstance.renderKnownTileAndWall())
            print("Robot now at the starting position, Congratulations!")
            moveTile.flashLED(stmInstance, loopCount=5, intervalSec=0.5, color=[255,255,255])  # Flash white LED to indicate completion
            LiDAR.liDARShutdown(lidarInstance)
            stmInstance.sts3032.stop()
            exit(0)
    except:
        import traceback
        traceback.print_exc()
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()               
if __name__ == "__main__":
    main()