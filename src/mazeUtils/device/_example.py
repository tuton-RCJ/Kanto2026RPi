import stm

myStm = stm.STM()

while True:
    myStm.update()
    print("--------------")
    print("Left UnitV:", myStm.unitv.status[stm.deviceEnums.Side.LEFT])
    print("Right UnitV:", myStm.unitv.status[stm.deviceEnums.Side.RIGHT])
    print(
        "Loadcell Value:",
        myStm.loadcell.getRawValue()[stm.deviceEnums.Side.LEFT],
        myStm.loadcell.getRawValue()[stm.deviceEnums.Side.RIGHT],
    )
    print(
        "Gyro :",
        myStm.gyro.getValue().heading,
        myStm.gyro.getValue().pitch,
        myStm.gyro.getValue().roll,
    )
    print("Switch:", myStm.switch.getPushSwitch1(), myStm.switch.getToggleSwitch1())
    # print(myStm.rescuekitservo.dropRescueKit(2, stm.deviceEnums.Side.LEFT))
    # time.sleep(1)
    # print(myStm.rescuekitservo.dropRescueKit(2, stm.deviceEnums.Side.RIGHT))
    # time.sleep(1)
    # myStm.sts3032.setMotorSpeed({stm.deviceEnums.Side.LEFT: 100, stm.deviceEnums.Side.RIGHT: 100})
    # time.sleep(1)
    # myStm.sts3032.setMotorSpeed({stm.deviceEnums.Side.LEFT: 0, stm.deviceEnums.Side.RIGHT: 0})
    # time.sleep(1)
    # myStm.led.setColor(255, 0, 0)
    # time.sleep(1)
    # myStm.led.setColor(0, 255, 0)
    # time.sleep(1)
    # myStm.led.setColor(0, 0, 255)
    # time.sleep(1)
    
    