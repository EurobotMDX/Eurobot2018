import RPi.GPIO as GPIO  # import RPi.GPIO module

GPIO.setmode(GPIO.BOARD)  # choose BCM or BOARD
GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


while True:
    print(GPIO.input(37))

