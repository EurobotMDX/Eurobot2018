import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(15, GPIO.OUT)

p = GPIO.PWM(15, 50)

p.start(7.5)

try:
    #while True:
    p.ChangeDutyCycle(6.5)  # turn towards 90 degree
    time.sleep(1) # sleep 1 second
    p.ChangeDutyCycle(5)  # turn towards 0 degree
    time.sleep(10) # sleep 1 second
    p.ChangeDutyCycle(6.5) # turn towards 180 degree
    time.sleep(1) # sleep 1 second

except KeyboardInterrupt:
	p.stop()
        GPIO.cleanup()

GPIO.cleanup()