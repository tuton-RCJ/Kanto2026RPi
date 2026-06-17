from mazeUtils.device.colorsensor import ColorSensor

color_sensor = ColorSensor()  # カラーセンサのポートを指定

try:
    while True:
        if color_sensor.update():
            r, g, b = color_sensor._colorRGB
            print(f"Color Sensor RGB: R={r}, G={g}, B={b}")
        else:
            print("Failed to read from Color Sensor.")
except KeyboardInterrupt:
    print("\nExiting...")