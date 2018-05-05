from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from time import sleep
import time
import datetime
# from servoControl import servoControl
# from Switches import *
# from Sensor import *
from settings import logging as log

if __name__ == '__main__':
    canRun = False

    sensorCenter = None
    sensorRight = None
    sensorLeft = None

    right = True
    left = False
    turnSpeed = 15

    try:
        robot = Driving()

        robot.changeAcc(1)

        if robot.checkStatus():
            canRun = True
            log.info("Initialized main objects")

        else:
            canRun = False

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if canRun:

        try:

            start_time = time.time()

            robot.driveRobot(distance=50, speed=20, sensors=[])

            robot.turnRobot(degrees=180, speed=20, direction=right)

            robot.driveRobot(distance=50, speed=20, sensors=[])

            robot.turnRobot(degrees=180, speed=20, direction=left)

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            print ("\n")

            log.info("All test pass")

            GPIO.cleanup()

    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)
