import RPi.GPIO as GPIO
import time
import sys
# sys.path.insert(0, '..')
# from settings import logging as log

# NOTE: don't use GPIO02 or GPIO03, they have
# internal pullup resistors that prevents results.


class Sensor:
    "This class provides capabilities to measure a distance from different sensors"
    def __init__(self, sensorPin, sensorPos="Default", frequency=0.1):
        self.sensorPin = sensorPin
        self.sensorPosition = sensorPos
        # self.frequency = frequency
        # self.measureRunning = True

    # Setup
    def setUp(self):
        try:
            GPIO.setmode(GPIO.BOARD)
            print("Successfully, set up sensor")
        except Exception as error:
            log.error("Failed setup for sensor")

    def readSensorValue(self):