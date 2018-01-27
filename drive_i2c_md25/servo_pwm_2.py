import RPi.GPIO as IO
import time
IO.setwarnings(False)
IO.setmode (IO.BCM)
IO.setup(19,IO.OUT)
p = IO.PWM(19,50)
p.start(7.5)
while 1:
        p.ChangeDutyCycle(7.5)
        time.sleep(1)
        p.ChangeDutyCycle(12.5)
        time.sleep(1)
        p.ChangeDutyCycle(2.5)
        time.sleep(1)