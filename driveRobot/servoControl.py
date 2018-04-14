# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

import sys
sys.path.insert(0, '..')
from settings import logging as log

class servoControl:

    def __init__(self, name, channel, frequency):

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        # Alternatively specify a different address and/or bus:
        #pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

        # Configure min and max servo pulse lengths
        self.servo_min = int(0.035 * 4096.0)  # Min pulse length out of 4096
        self.servo_max = int(0.1450 * 4096.0)  # Max pulse length out of 4096

        # Channel settings
        self.channel = channel

        # Servo settings

        self.maxDegrees = 180

        self.minDegrees = 0

        self.pwm.set_pwm_freq(frequency)

        self.name = name

        log.debug("Initialized servo {}".format(name))

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(self, channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        # pulse_length //= 60       # 60 Hz
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        pwm.set_pwm(channel, 0, pulse)

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def turn(self, degrees):

        newValueDegrees = int(self.translate(degrees, self.minDegrees, self.maxDegrees, self.servo_min, self.servo_max,))

        self.pwm.set_pwm(self.channel, 0, newValueDegrees)

        log.debug("Servo: '{}' turned to {} degrees".format(self.name, degrees))


    def testRaw(self):

        while True:

            print("Going to: {}".format(self.servo_min))
            self.pwm.set_pwm(self.channel, 0, self.servo_min)
            time.sleep(3)
            print("Going to: {}".format(self.servo_max))
            self.pwm.set_pwm(self.channel, 0, self.servo_max)
            time.sleep(3)


    def checkPWM(self, pwmValue):

        self.pwm.set_pwm(self.channel, 0, pwmValue)
        print("PWM value: {}".format(pwmValue))


if __name__ == '__main__':

    # Init servo
    servo = servoControl("main", 0, 60)

    # Set frequency to 60hz, good for servos.

    # servo.setFrequency(50)

    print('Moving servo on channel 0, press Ctrl-C to quit...')
    while True:
        # Move servo on channel O between extremes.
        servo.degress(0)
        # pwm.set_pwm(0, 0, servo_min)
        time.sleep(3)
        servo.degress(100)
        # pwm.set_pwm(0, 0, servo_max)
        time.sleep(3)
