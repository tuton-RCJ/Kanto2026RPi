from mazeUtils.device.arduinoNanoEvery import ArduinoNanoEveryUART

arduino = ArduinoNanoEveryUART(port="/dev/ttyACM0")

distance_mm = arduino.request_tof_distance_mm()
ok = arduino.update_oled(x_coord=3, y_coord=5, direction=1)  # 0:N,1:E,2:S,3:W