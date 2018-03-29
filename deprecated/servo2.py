# import RPi.GPIO as IO
# import time
# IO.setwarnings(False)
# IO.setmode (IO.BCM)
# IO.setup(19,IO.OUT)
# p = IO.PWM(19,50)
# p.start(7.5)
# while 1:
#         p.ChangeDutyCycle(7.5)
#         time.sleep(1)
#         p.ChangeDutyCycle(12.5)
#         time.sleep(1)
#         p.ChangeDutyCycle(2.5)
#         time.sleep(1)
#
#

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)

p.start(7.5)

def test():
    try:
        while True:
            p.ChangeDutyCycle(7.5)  # turn towards 90 degree
            time.sleep(1) # sleep 1 second
            p.ChangeDutyCycle(2.5)  # turn towards 0 degree
            time.sleep(1) # sleep 1 second
            p.ChangeDutyCycle(12.5) # turn towards 180 degree
            time.sleep(1) # sleep 1 second
    except KeyboardInterrupt:
            p.stop()
            GPIO.cleanup()