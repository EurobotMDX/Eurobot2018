#!/usr/bin/env python
from math import pi

# Constants for robot

# Diameter of wheels in mm
wheelDiameter = 100

# Encoder values for one full revolution of the wheel
oneRevolution = 360

# Space  between wheels
wheelsSpacing = 25.9


# Robot types:
# main or secondary

robotSettings = {'robotType': 'main',
                 'oneEncMM': 2 * pi * (wheelDiameter / 2) / oneRevolution / 10,
                 'circumferenceOfCircle': 2 * pi * (wheelsSpacing / 2),
                 'sensorThreshold': 20,
                 }

debugMode = True