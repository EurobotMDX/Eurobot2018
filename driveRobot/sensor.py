import RPi.GPIO as GPIO
import time
import sys
sys.path.insert(0, '..')
from settings import logging as log

# NOTE: don't use GPIO02 or GPIO03, they have
# internal pullup resistors that prevents results.


class Sensor:
    "This class provides capabilities to measure a distance from different sensors"
    def __init__(self, sensorPin, sensorPos="Front", frequency=0.1):
        self.sensorPin = sensorPin
        self.sensorPosition = sensorPos
        self.frequency = frequency
        self.measureRunning = True

    # Setup
    def setUp(self):
        try:
            GPIO.setmode(GPIO.BOARD)
            print("Successfully, set up sensor")
        except Exception as error:
            log.error("Failed setup for sensor")

    def getSensorValue(self):
        sensorPin = self.sensorPin

        GPIO.setup(sensorPin, GPIO.OUT)
        # Set to low
        GPIO.output(sensorPin, False)

        # Sleep 2 micro-seconds
        time.sleep(0.000002)

        # Set high
        GPIO.output(sensorPin, True)

        # Sleep 5 micro-seconds
        time.sleep(0.000005)

        # Set low
        GPIO.output(sensorPin, False)

        # Set to input
        GPIO.setup(sensorPin, GPIO.IN)
        starttime = 0
        # Count microseconds that SIG was high
        while GPIO.input(sensorPin) == 0:
          starttime = time.time()

        # TODO changed to none object, do some testing to make sure is not breaking anything
        endtime = time.time()

        while GPIO.input(sensorPin) == 1:
          endtime = time.time()

        duration = endtime - starttime
        # The speed of sound is 340 m/s or 29 microseconds per centimeter.
        # The ping travels out and back, so to find the distance of the
        # object we take half of the distance travelled.
        # distance = duration / 29 / 2
        distance = duration * 34000 / 2

        # print (distance)

        time.sleep(self.frequency)

        return distance
        # except Exception as error:
        #     log.error("Error for sensor: '%s'. Pin number: %s" % (sensorPosition, sensorPin))

    def stopSensor(self):
        GPIO.cleanup()
        print("Sensor stopped! Cleanup done!")
