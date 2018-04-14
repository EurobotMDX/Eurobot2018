#!/usr/bin/env python
from math import pi
import logging
import datetime
import logging.config

# Constants for robot

# Diameter of wheels in mm
wheelDiameter = 100

# Encoder values for one full revolution of the wheel
oneRevolution = 360

# Space  between wheels
wheelsSpacing = 25.9

# Robot types:
# main or secondary

robotSettings = {
    'robotType': 'main',
    'oneEncMM': 2 * pi * (wheelDiameter / 2) / oneRevolution / 10,
    'circumferenceOfCircle': 2 * pi * (wheelsSpacing / 2),
    'sensorThreshold': 10,
}

debugMode = True


# Logger settings
logging.root.handlers = []

FORMAT = '%(asctime)s : %(levelname)s : %(message)s\r'

logging.basicConfig(format=FORMAT, level=logging.DEBUG,
                    filename='../logs.log'
                    )

# set up logging to console
console = logging.StreamHandler()
console.setLevel(logging.DEBUG)  # this is only if we want to error logs be printed out to console

# set a format which is simpler for console use
formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(message)s\r')
console.setFormatter(formatter)
logging.getLogger("").addHandler(console)

logger = logging.getLogger('Adafruit_I2C.Device')
logger.setLevel(logging.CRITICAL)
