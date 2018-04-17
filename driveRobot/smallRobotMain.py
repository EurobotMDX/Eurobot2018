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

def openArms():
    rightArm.turn(degrees=120)
    sleep(0.5)
    leftArm.turn(degrees=52)
    sleep(1)


def closeArms():
    leftArm.turn(degrees=190)
    sleep(0.5)
    rightArm.turn(degrees=0)
    sleep(1)

if __name__ == '__main__':
    canRun = False

    try:
        robot = Driving()

        # Initialize servos
        rightArm = servoControl("Right Arm", 15, 60)
        leftArm = servoControl("Left Arm", 13, 60)
        switchArm = servoControl("Switch Arm", 14, 60)

        log.info("Initialized main objects")

        canRun = True
        turnSpeed = 10


    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if robot.checkStatus() and canRun:

        log.debug("Expecting for start switch")
        while startSwitch():
            pass

        try:
            thread.start_new_thread(extra.timer, (robot,))
            log.info("Start timer thread")

        except Exception as error:
            log.error("Error: unable to start thread %s" %error)

        try:
            start_time = time.time()

            if sideSwitch() == "Orange":
                print ("Orange site")

            if sideSwitch() == "Green":
                print ("Green site")

            openArms()

            closeArms()

            robot.driveRobot(distance=100, speed=10, sensors=[1, 2])

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

