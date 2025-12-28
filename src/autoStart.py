#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import subprocess

def set_wireless_state(enabled):
    state = "unblock" if enabled else "block"
    subprocess.run(["sudo", "rfkill", state, "wifi"], check=True)

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(1)  
    if GPIO.input(26) == 1:
        set_wireless_state(False)
        subprocess.run(["/home/tuton/Kanto2026RPi/venv/bin/python3","/home/tuton/Kanto2026RPi/src/main.py"])
    else:
        set_wireless_state(True)


if __name__ == "__main__":
    print("hello")
    main()
print("Oh")