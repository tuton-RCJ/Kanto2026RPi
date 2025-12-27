import RPi.GPIO as GPIO
import time
import subprocess

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(1)  
    if GPIO.input(26) == 1:
        subprocess.run(["/home/tuton/Kanto2026RPi/venv/bin/python3","/home/tuton/Kanto2026RPi/src/main.py"])



if __name__ == "__main__":
    main()