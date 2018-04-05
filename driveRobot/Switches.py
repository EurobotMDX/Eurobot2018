import RPi.GPIO as GPIO  # import RPi.GPIO module

GPIO.setmode(GPIO.BOARD)  # choose BCM or BOARD
GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(35, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def startSwitch():
    return GPIO.input(37)

def sideSwitch():
    if GPIO.input(35):
        return "Orange"
    else:
        return "Green"