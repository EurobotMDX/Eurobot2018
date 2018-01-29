import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
pwm=GPIO.PWM(11, 50)
pwm.start(0)

def SetAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(11, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(11, False)
    pwm.ChangeDutyCycle(0)

SetAngle(0)
SetAngle(90)
SetAngle(5)
SetAngle(45)
SetAngle(0)
SetAngle(90)
SetAngle(0)
SetAngle(180)




pwm.stop()
GPIO.cleanup()
