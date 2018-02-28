import RPi.GPIO as GPIO
from time import sleep
from settings import logging as log

class Servo():
    def __init__(self, pinNumber, name="Default"):
        self.name = name
        self.pin = pinNumber
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, 50)
        self.servo.start(7.0)
        log.info("Successful set up for servo!")

    def setAngle(self, angle):
        self.servo.ChangeDutyCycle(angle)

    def cleanup(self):
        log.info("Servo cleanup done!")
        self.servo.stop()
        GPIO.cleanup()
