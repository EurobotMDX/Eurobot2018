from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from settings import logging as log
from time import sleep
import datetime
from servoControl import servoControl
from valve import *

# old servo
# from servo import Servo


if __name__ == '__main__':
    canRun = False

    try:
        robot = driving()
        # Initialize servos

        servoArm = servoControl("Arm servo", 0, 60)
        servoPipe = servoControl("Pipe servo", 1, 60)
        servoBee = servoControl("Bee servo", 2, 60)

        extra = robotHelpers()

        log.info("Initialized main objects")

        canRun = True

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if robot.checkStatus() and canRun:

        try:

            robot.turnRobot(180, 20, False)

            servoArm.turn(0)
            servoArm.turn(90)

            sleep(2)

            servoArm.turn(0)

            sleep(2)

            extra.motorsOn()

            sleep(2)

            extra.motorsOff()

            sleep(2)

            extra.valveRelease()

            robot.turnRobot(180, 20, False)

            sleep(2)

            extra.motorsOn()

            sleep(2)

            extra.motorsOff()

            sleep(2)

            extra.valveRelease()

            # robot.driveBack(30, 10, False)

            # sleep(0.5)
            # Delay between is used in order to make sure that encoders were reset completely.
            # robot.driveRobot(100, 50)
            # robot.turnRobot(180, 15, False)
            # sleep(0.5)
            # sleep(10)



        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")
            GPIO.cleanup()
    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

