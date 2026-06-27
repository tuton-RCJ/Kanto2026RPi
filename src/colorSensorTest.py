from mazeUtils.device.colorsensor import ColorSensor

color_sensor = ColorSensor()  # カラーセンサのポートを指定

try:
    while True:
        if color_sensor.update():
            r, g, b = color_sensor._colorRGB
            rf1,rf2 = color_sensor._reflectance
            print(f"Color Sensor RGB: R={r}, G={g}, B={b}, Rf1={rf1}, Rf2={rf2}")
        else:
            print("Failed to read from Color Sensor.")
except KeyboardInterrupt:
    print("\nExiting...")