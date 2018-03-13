import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(15, GPIO.OUT)
GPIO.setwarnings(False)
pwm=GPIO.PWM(15, 50)
leftPosition=0.5
rightPosition=2.5
middlePosition=(rightPosition-leftPosition)/2+leftPosition
positionList=[leftPosition, middlePosition, rightPosition, middlePosition]
msPerCycle=1000/50

for i in range(20):
    for position in positionList:
        dutyCyclePercentage=position * 100 / msPerCycle
        print ("Position:" +str(position))
        print ("Duty Cycle:" +str(dutyCyclePercentage)+"%")
        pwm.start(dutyCyclePercentage)
        time.sleep(1)
pwm.stop()
GPIO.cleanup()