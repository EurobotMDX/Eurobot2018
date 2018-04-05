import RPi.GPIO as GPIO
from time import sleep
# from settings import logging as log

class Servo():
    def __init__(self, pinNumber, name="Default"):
        self.name = name
        self.pin = pinNumber
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, 50)
        self.servo.start(7.0)
        # log.info("Successful set up for servo!")

    def setAngle(self, angle):
        self.servo.ChangeDutyCycle(angle)

    def cleanup(self):
        # log.info("Servo cleanup done!")
        self.servo.stop()
        GPIO.cleanup()

# !/usr/bin/env python

# from time import sleep  # Allows us to call the sleep function to slow down our loop
# import RPi.GPIO as GPIO  # Allows us to call our GPIO pins and names it just GPIO

# GPIO.setmode(GPIO.BOARD)  # Set's GPIO pins to BCM GPIO numbering
# INPUT_PIN = 4  # Sets our input pin, in this example I'm connecting our button to pin 4. Pin 0 is the SDA pin so I avoid using it for sensors/buttons
# GPIO.setup(INPUT_PIN, GPIO.IN)  # Set our input pin to be an input
#
#
# # Create a function to run when the input is high
# def inputLow(channel):
#     print('0')
#
#
# GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=inputLow,
#                       bouncetime=200)  # Wait for the input to go low, run the function when it does
#
# # Start a loop that never ends
# while True:
#     print('3.3')
#     sleep(1)