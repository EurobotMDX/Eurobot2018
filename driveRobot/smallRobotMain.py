from robotFunctions import *
from terminalColors import bcolors as tc
import sys

sys.path.insert(0, '..')
from settings import logging as log
from time import sleep
import time
import datetime
from servoControl import servoControl
from Switches import *


if __name__ == '__main__':
    canRun = False

    try:
        robot = Driving(True)

        # Initialize servos
        servo1 = servoControl("Servo 1", 15, 60)
        # servoPipe = servoControl("Pipe servo", 1, 60)
        # servoBee = servoControl("Bee servo", 2, 60)

        # sensorCenter = Sensor(0x72, "centre")
        # sensorRight = Sensor(0x71, "right")
        # sensorLeft = Sensor(0x73, "left")

        log.info("Initialized main objects")

        canRun = True

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if robot.checkStatus() and canRun:

        log.debug("Expecting for start switch")
        while not startSwitch():
            pass

        sleep(0.5)

        try:

            if sideSwitch() == "Orange":
                print ("Orange site")

            if sideSwitch() == "Green":
                print ("Green site")

            turnSpeed = 10

            start_time = time.time()

            servo1.turn(degrees=90)

            robot.driveRobot(distance=53, speed=15, sensors=[])

            sleep(0.5)

            robot.turnRobot(degrees=45, speed=5, direction=left)


        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")
            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            GPIO.cleanup()

    else:
        log.info(
            tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

