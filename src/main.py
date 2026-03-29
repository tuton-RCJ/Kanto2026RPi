from mazeUtils import mazeMap
from mazeUtils import moveTile
from mazeUtils.device import stm
from mazeUtils.device import LiDAR
from mazeUtils.device import buzzerSongs
from mazeUtils import mazeEnums
from mazeUtils import mazeConstraints
from mazeUtils.device import deviceEnums
import ydlidar
import time

def main():
    stmInstance = stm.STM()
    stmInstance.update()
    mapInstance = mazeMap.mazeMap()

    mapInstance.arduinoNanoEvery.camled((0, 0, 0)) 
    lidarInstance = LiDAR.initializeLidar()
    stmInstance.buzzer.playMusic(buzzerSongs.start)    
    stmInstance.update()
    while stmInstance.switch.getToggleSwitch1():
        stmInstance.update()    
    stmInstance.gyro.setOffset(stmInstance.gyro.getValue())
    time.sleep(1) 
    try:
        while True:
            stmInstance.update()
            moveTile.detectWall(lidarInstance, mapInstance)
            mapInstance.saveCache()
            print("Initial Map:")
            print(mapInstance.renderKnownTileAndWall())
            
            nextDirection = mapInstance.getNearestUnexploredTile()
            print(f"Next Direction: {nextDirection}")
            print("Exploration started.")
            stopped = False
            while nextDirection is not None or stopped:
                nextDirection = mapInstance.getNearestUnexploredTile()
                if nextDirection is None:
                    continue
                for direction in nextDirection:
                    isBlack, stopped = moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    print(mapInstance.renderKnownTileAndWall())
                    toggleswitchFlag = False
                    stmInstance.update()

                    if stmInstance.switch.getToggleSwitch1():
                        print("Exploration paused. Toggle switch 1 to resume.")

                    while stmInstance.switch.getToggleSwitch1():
                        stmInstance.update()
                        toggleswitchFlag = True
                    print(toggleswitchFlag)
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
                        stmInstance.update()
                        nextDirection = mapInstance.getNearestUnexploredTile()
                        break
                    moveTile.flashLED(stmInstance, mapInstance, loopCount=1, intervalSec=0, color=[0,0,0]) 

                nextDirection = mapInstance.getNearestUnexploredTile()
            stmInstance.buzzer.playMusic(buzzerSongs.hotaru)       
            returnPath = mapInstance.getPathTo((20, 20))
            print(f"Return Path: {returnPath}")
            if returnPath:
                for direction in returnPath:
                    moveTile.moveNextTile(direction, mapInstance, stmInstance, lidarInstance)
                    toggleswitchFlag = False
                    stmInstance.update()
                    if stmInstance.switch.getToggleSwitch1():
                        print("Return to start paused. Toggle switch 1 to resume.")
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
                        stmInstance.update()
                        moveTile.detectWall(lidarInstance, mapInstance)
                        tileType = moveTile.detectTileColor()
                        mapInstance.setTileType(tileType)
                        moveTile.rescueVictim(mapInstance, stmInstance)
                        moveTile.flashLED(stmInstance, mapInstance, loopCount=1, intervalSec=0, color=[0,0,0]) 
                        break
                    print(mapInstance.renderKnownTileAndWall())
            else:
                print("Robot now at the starting position, Congratulations!")
                stmInstance.buzzer.playMusic(buzzerSongs.matuken) 
                moveTile.flashLED(stmInstance, mapInstance, loopCount=5, intervalSec=1, color=[255,255,255])  # Flash white LED to indicate completion
                stmInstance.sts3032.stop()
                print(mapInstance.renderKnownTileAndWall())
                while not stmInstance.switch.getToggleSwitch1():
                    stmInstance.update()
                print("detect LoP. back to last check point.")
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
                    stmInstance.update()
                    moveTile.detectWall(lidarInstance, mapInstance)
                    tileType = moveTile.detectTileColor()
                    mapInstance.setTileType(tileType)
                    moveTile.rescueVictim(mapInstance, stmInstance)
                    moveTile.flashLED(stmInstance, mapInstance, loopCount=1, intervalSec=0, color=[0,0,0]) 
            continue
    except:
        LiDAR.liDARShutdown(lidarInstance)
        stmInstance.sts3032.stop()      
        moveTile.flashLED(stmInstance, mapInstance, loopCount=1, intervalSec=0, color=[0,0,0])  # Flash red LED to indicate error     
        import traceback
        traceback.print_exc()   
if __name__ == "__main__":
    main()