import RPi.GPIO as GPIO  # import RPi.GPIO module
from time import sleep  # lets us have a delay

pin = 29
pin1 = 31

GPIO.setmode(GPIO.BOARD)  # choose BCM or BOARD
GPIO.setup(pin, GPIO.OUT)  # set GPIO24 as an output
GPIO.setup(pin1, GPIO.OUT)


# def valve():
#
#     try:
#         while True:
#             GPIO.output(pin, 1)  # set GPIO24 to 1/GPIO.HIGH/True
#             sleep(0.5)  # wait half a second
#             GPIO.output(pin, 0)  # set GPIO24 to 0/GPIO.LOW/False
#             sleep(0.5)  # wait half a second
#
#     except KeyboardInterrupt:  # trap a CTRL+C keyboard interrupt
#         GPIO.cleanup()
#
#

def motorOn():
    GPIO.output(pin, 1)
    print("Start motors")

def motorOff():
    GPIO.output(pin, 0)
    print("Motor stopped")

def valveRelease():
    k = 0
    while k < 20:
        GPIO.output(pin1, 1)
        sleep(0.03)
        GPIO.output(pin1, 0)
        sleep(0.04)
        k += 1

    sleep(1)

    GPIO.output(pin1, 1)
    sleep(0.05)
    GPIO.output(pin1, 0)
    sleep(0.06)
    k += 1
