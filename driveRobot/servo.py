import RPi.GPIO as GPIO
from time import sleep
from settings import logging as log

class Servo():
    def __init__(self, pinNumber, name="Default"):
        self.name = name
        self.pin = pinNumber
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)
        log.info("Successful set up for servo!")

    def setAngle(self, angle):
        duty = angle / 18 + 2
        # GPIO.output(self.pin, True)
        # self.pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(self.pin, False)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        log.info("Servo cleanup done!")
        self.pwm.stop()
        GPIO.cleanup()
