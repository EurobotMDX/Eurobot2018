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


# old servo
# from servo import Servo


if __name__ == '__main__':
    canRun = False

    try:
        robot = Driving()
        # Initialize servos

        servoArm = servoControl("Arm servo", 0, 60)
        servoPipe = servoControl("Pipe servo", 1, 60)
        servoBee = servoControl("Bee servo", 2, 60)

        extra = RobotHelpers()

        log.info("Initialized main objects")

        canRun = True

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    # TODO enable this function 
    # if robot.checkStatus() and canRun:
    if canRun:

        while not startSwitch():
            print(sideSwitch())
            pass

        sleep(0.5)

        try:

            turnSpeed = 15

            start_time = time.time()

            # robot.sensorTest(60)

            for i in range(0, 0):

                robot.turnRobot(90, turnSpeed, False)
                sleep(0.3)

            robot.driveRobot(100, 20)

            # extra.valveRelease()

            #
            # robot.turnRobot(90, turnSpeed, True)
            #
            # sleep(2)
            #
            # robot.turnRobot(90, turnSpeed, False)
            #
            # sleep(2)
            #
            # robot.turnRobot(180, turnSpeed, False)
            #
            # sleep(2)
            #
            # robot.turnRobot(180, turnSpeed, True)
            #
            # sleep(2)
            #
            # robot.turnRobot(360, turnSpeed, True)

            # robot.driveRobot(50, turnSpeed)

            # servoArm.turn(0)
            # servoArm.turn(90)
            '''
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

        '''
            # robot.driveBack(30, turnSpeed, False)

            # sleep(0.5)
            # Delay between is used in order to make sure that encoders were reset completely.
            # robot.driveRobot(turnSpeed0, 50)
            # robot.turnRobot(180, 15, False)
            # sleep(0.5)
            # sleep(turnSpeed)



        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")
            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            GPIO.cleanup()

    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

