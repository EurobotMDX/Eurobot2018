import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
# GPIO.setmode(GPIO.BCM)


TRIG1 = 18
ECHO1 = 16

TRIG2 = 22
ECHO2 = 24

# print "Distance Measurement In Progress"

GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

# print "Waiting For Sensor To Settle"
# time.sleep(0.3)

def getSensorHCValue(sensor):
    if sensor == 1:
        TRIG = TRIG1
        ECHO = ECHO1

    elif sensor == 2:
        TRIG = TRIG2
        ECHO = ECHO2

    try:
        # while True:
        GPIO.output(TRIG, False)
        time.sleep(0.00001)

        # Start here
        GPIO.output(TRIG, True)
        time.sleep(0.01)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          pulse_start = time.time()

        while GPIO.input(ECHO)==1:
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

        return distance

    except Exception as error:
        print ("Error while reading a sensor value")

def sensorCleanUp():
    print ("Finish sensor reading, GPIO cleanup")
    GPIO.cleanup()