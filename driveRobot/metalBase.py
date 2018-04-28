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
<<<<<<< HEAD

        log.info("Initialized main objects")

        if robot.checkStatus():
            canRun = True
        else:
            canRun = False
=======
        # Initialize servos

        log.info("Initialized main objects")

        canRun = True
>>>>>>> b11c8671361f15a4c60cdd1d8581c38248923cf1

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

<<<<<<< HEAD
    if canRun:

        try:

            start_time = time.time()

            robot.driveRobot(distance=150, speed=50, sensors=[])

            sleep(2)

            robot.turnRobot(degrees=180, speed=10, direction=left)

            sleep(1)

            robot.driveRobot(distance=150, speed=50, sensors=[])

            sleep(1)

            robot.turnRobot(degrees=180, speed=10, direction=left)

=======
    if robot.checkStatus() and canRun:

        try:
            start_time = time.time()
            sleep(1)

            # robot.turnRobot(degrees=20, speed=15, direction=right)

            robot.driveRobot(distance=5, speed=5, sensors=[])
>>>>>>> b11c8671361f15a4c60cdd1d8581c38248923cf1


        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")

            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            print ("\n")

            log.info("All test pass")

            GPIO.cleanup()

    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)
