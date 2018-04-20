import RPi.GPIO as GPIO  # import RPi.GPIO module

sys.path.insert(0, '..')
import settings as config

startSwitchPin = config.robotSettings['startSwitchPin']
sideSwitchPin = config.robotSettings['sideSwitchPin']

GPIO.setmode(GPIO.BOARD)  # choose BCM or BOARD
GPIO.setup(startSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(sideSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def startSwitch():
    return GPIO.input(startSwitchPin)

def sideSwitch():
    if GPIO.input(sideSwitchPin):
        return "Orange"
    else:
        return "Green"