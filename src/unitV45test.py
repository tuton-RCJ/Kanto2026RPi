from mazeUtils.device import stm
from mazeUtils.device import deviceEnums


try:
    stmInstance = stm.STM()
    while True:
        a = int(input())
        if a==0:
            stmInstance.unitv.set45Mode(False)
        elif a==1:
            stmInstance.unitv.set45Mode(True)
        else:
            stmInstance.update()
            vic = stmInstance.unitv.getStatus()
            print(vic[deviceEnums.Side.LEFT],vic[deviceEnums.Side.RIGHT])
            print(stmInstance.unitv.getLastUpdateTime())
            
            
except KeyboardInterrupt:
    print("finish")