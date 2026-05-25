import RPi.GPIO as GPIO

class PhotoReflector:
    def __init__(self, pin: int = 4) -> None:
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def isReflecting(self) -> bool:
        return False
        return GPIO.input(self.pin) == GPIO.HIGH