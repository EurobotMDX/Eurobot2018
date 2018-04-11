from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from settings import logging as log
from time import sleep
import time
import datetime
from Switches import *
# from servoControl import servoControl

turnLeft = False;
turnRight = True;

if __name__ == '__main__':
    canRun = False

    try:
        robot = Driving(False)



        # Initialize servos

        # servoArm = servoControl("Arm servo", 0, 60)
        # servoPipe = servoControl("Pipe servo", 1, 60)
        # servoBee = servoControl("Bee servo", 2, 60)

        # extra = RobotHelpers()

        log.info("Initialized main objects")

        canRun = True

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    # TODO enable this function
    # if robot.checkStatus() and canRun:
    if canRun:

        log.debug("Expecting for start switch")
        # while not startSwitch():
        #     print(sideSwitch())
            # pass

        sleep(0.5)

        try:
            turnSpeed = 100

            start_time = time.time()

            # if sideSwitch() == "Orange":
            #     robot.turnRobot(90, turnSpeed, False)

            # else:
            #     robot.turnRobot(90, turnSpeed, True)

            sleep(0.3)

            # robot.driveRobot(69, 30)

            # sleep(0.3)
            #
            #robot.turnRobot(360, turnSpeed, turnLeft)
            # sleep(0.3)

            # extra.motorsOn()

            # Air valve here
            # sleep(10)

            # extra.motorsOff()

            # robot.turnRobot(90, turnSpeed, True)
            #
            # sleep(0.3)
            #
            # robot.driveRobot(115, 30)
            #
            # sleep(0.3)
            #
            # log.info("Bee deployment")

            # Bee deployment
            # servoBee.turn(0)

            # robot.turnRobot(90, turnSpeed, True)
            #
            # sleep(2)
            #
            # print("Servo operations")
            #
            # if sideSwitch() == "Orange":
            #
            #     robot.driveRobot(23, 30)
            #
            #     sleep(0.3)
            #
            #     log.info("Bee closing servo")
            #
            # else:
            #     log.info("Bee open servo")
            #
            # robot.turnRobot(90, turnSpeed, True)
            #
            # sleep(0.3)
            #
            # robot.driveRobot(18, 30)
            #
            # sleep(0.3)
            #
            # if sideSwitch() == "Orange":
            #
            #     robot.turnRobot(135, turnSpeed, False)
            #
            # else:
            #     robot.turnRobot(135, turnSpeed, True)
            #
            # sleep(0.3)
            #
            # robot.driveRobot(25, 30)
            #
            # sleep(0.3)
            #
            # log.info("Deploy servo for pipe")
            #
            # if sideSwitch() == "Orange":
            #
            #     robot.turnRobot(45, turnSpeed, False)
            #
            # else:
            #     robot.turnRobot(45, turnSpeed, True)
        #
        #


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

