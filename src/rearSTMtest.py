from config import setup_logging, get_logger
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
    try: 
        stmInstance = stm.STM()
        while True:
            a = input()
            # stmInstance.rearSTM.victimled((255, 0, 0-))
            # stmInstance.rearSTM.update_oled(5,8,int(a),int(a))
            # stmInstance.rearSTM.send_message(a)
            stmInstance.rearSTM.playMusic(buzzerSongs.swamp)
    except KeyboardInterrupt:
        print("rearSTMtest.py: rearSTMを終了しました。")
        
if __name__ == "__main__":
    main()