import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_rgb_display.ili9341 import ILI9341
from adafruit_rgb_display.rgb import color565
import time

# Pin Config
cs_pin = DigitalInOut(board.CE1) # 26番ピン
dc_pin = DigitalInOut(board.D25) # 22番ピン
rst_pin = DigitalInOut(board.D8) # 24番ピン

spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)

# Reset sequence
rst_pin.direction = Direction.OUTPUT
rst_pin.value = False
time.sleep(0.2)
rst_pin.value = True
time.sleep(0.2)

display = ILI9341(spi, cs=cs_pin, dc=dc_pin, rst=rst_pin, baudrate=1000000)

print("Filling display with Blue...")
display.fill(color565(0, 0, 255))

while True:
    time.sleep(1) # プログラムを終了させない