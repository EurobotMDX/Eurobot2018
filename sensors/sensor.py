import RPi.GPIO as GPIO
import time
<<<<<<< HEAD
import threading


GPIO.setmode(GPIO.BOARD)

#leftTrig = 21
#leftEcho = 22

#rightTrig = 19
#rightEcho = 20

front_leftTrig = 11
front_leftEcho = 12

GPIO.setup(front_leftTrig, GPIO.OUT)
GPIO.output(front_leftTrig, 0)

GPIO.setup(front_leftEcho, GPIO.IN)

front_rightTrig = 16
front_rightEcho = 18

GPIO.setup(front_rightTrig, GPIO.OUT)
GPIO.output(front_rightTrig, 0)

GPIO.setup(front_rightEcho, GPIO.IN)

frequency = 0.1


print("Starting measurment..")

def front_leftSensor(in_time=25):

    readTime = time.time() + in_time

    while (readTime >= time.time()):

        time.sleep(0.1)

        GPIO.output(front_leftTrig, 1)

        time.sleep(0.00001)

        GPIO.output(front_leftTrig, 0)

        while GPIO.input(front_leftEcho) == 0:
            pass

        start1 = time.time()

        while GPIO.input(front_leftEcho) == 1:
            pass

        stop1 = time.time()

        print ("Left: " + str((stop1 - start1) * 17000))
        time.sleep(frequency)


def front_rightSensor(in_time=25):

    readTime = time.time() + in_time

    while (readTime >= time.time()):
        time.sleep(0.1)

        GPIO.output(front_rightTrig, 1)

        time.sleep(0.00001)

        GPIO.output(front_rightTrig, 0)

        while GPIO.input(front_rightEcho) == 0:
            pass

        start2 = time.time()

        while GPIO.input(front_rightEcho) == 1:
            pass

        stop2 = time.time()
        print ("Right: " + str((stop2 - start2) * 17000) + "\n")
        time.sleep(frequency)




t1 = threading.Thread(target=front_leftSensor)
t2 = threading.Thread(target=front_rightSensor)
t1.start()
t2.start()
=======

GPIO.setmode(GPIO.BOARD)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, 0)


GPIO.setup(ECHO, GPIO.IN)

time.sleep(0.1)

print ("starting measurment..")

GPIO.output(TRIG,1)
time.sleep(0.00001)


GPIO.output(TRIG,0)

while GPIO.input(ECHO) == 0:
        pass

start = time.time()
print("here")

while GPIO.input(ECHO) == 1:
        pass

stop = time.time()

print ((stop - start) * 17000)

GPIO.cleanup()
>>>>>>> master
