import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

TRIG = 11
ECHO = 12

GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, 0)

GPIO.setup(ECHO, GPIO.IN)

print("starting measurment..")

e4=time.time() + 25

while (e4 >= time.time()):
    time.sleep(0.1)


    GPIO.output(TRIG, 1)
    time.sleep(0.00001)

    GPIO.output(TRIG, 0)

    while GPIO.input(ECHO) == 0:
        pass

    start = time.time()

    while GPIO.input(ECHO) == 1:
        pass

    stop = time.time()
    print ((stop - start) * 17000)
    time.sleep(1)



# print ((stop - start) * 17000)

GPIO.cleanup()