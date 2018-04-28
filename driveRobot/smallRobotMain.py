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
import settings as config


sleepAfterEachOperation = 1
left = False
right = True

turnSpeed = 10


#
def openRight():
    rightArm.turn(120)
    sleep(sleepAfterEachOperation)


def openLeft():
    leftArm.turn(52)
    sleep(sleepAfterEachOperation)


def orangeSide():
    sleep(sleepAfterEachOperation)
    openLeft()
    sleep(sleepAfterEachOperation)
    robot.driveRobot(distance=55, speed=20, sensors=[1, 2])
    sleep(sleepAfterEachOperation)
    # Start turning
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.driveRobot(distance=50, speed=20, sensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=right, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.driveRobot(distance=85, speed=20, sensors=[1, 2])


def greenSide():
    sleep(sleepAfterEachOperation)
    openLeft()
    sleep(sleepAfterEachOperation)
    robot.driveRobot(distance=55, speed=20, sensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.driveRobot(distance=50, speed=20, sensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)
    robot.turnRobot(degrees=15, speed=turnSpeed, direction=left, smallRobotSensors=[1, 2])
    sleep(sleepAfterEachOperation)

    robot.driveRobot(distance=85, speed=20, sensors=[1, 2])


if __name__ == '__main__':
    canRun = False

    rightArmServoChannel = config.robotSettings['rightArmServoChannel']
    leftArmServoChannel = config.robotSettings['leftArmServoChannel']

    try:  #
        robot = Driving()

        rightArm = servoControl("Right Arm", rightArmServoChannel, 60)
        leftArm = servoControl("Left Arm", leftArmServoChannel, 60)

        log.info("Initialized main objects")

        if robot.checkStatus():
            canRun = True
        else:
            canRun = False

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if canRun:

        sleep(1)

        log.debug("Expecting for start switch")

        while startSwitch():
            pass

        try:
            start_time = time.time()

            if sideSwitch() == "Orange":
                print ("Orange site")
                sleep(1)
                openLeft()
                orangeSide()

            if sideSwitch() == "Green":
                print ("Green site")
                sleep(1)
                openRight()
                greenSide()



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

