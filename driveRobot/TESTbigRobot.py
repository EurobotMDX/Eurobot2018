from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from time import sleep
import time
import datetime
from servoControl import servoControl
from Switches import *
from Sensor import *
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
        # Initialize servos

        servoPipe = servoControl("Pipe servo", 0, 60)
        servoArm = servoControl("Arm servo", 1, 60)
        servoBee = servoControl("Bee servo", 2, 60)

        sensorCenter = Sensor(0x72, "centre")
        sensorRight = Sensor(0x71, "right")
        sensorLeft = Sensor(0x73, "left")

        sensors = [sensorLeft, sensorCenter, sensorRight]

        extra = RobotHelpers()

        log.info("Initialized main objects")

        canRun = True

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    # TODO enable this function
    if robot.checkStatus() and canRun:
    # if canRun:
        if sideSwitch() == "Orange":
            log.info("Set 'Orange servo init, position: 8")

            servoPipe.turn(98)
        else:
            log.info("Set 'Orange servo init, position: 8")

            servoPipe.turn(8)

        log.debug("Expecting for start switch")

        log.debug("Side selection switch value: %s" % sideSwitch())

        # extra.motorsOff()

        # extra.motorsOff()
        # extra.valveRelease()

        while not startSwitch():
            pass

        sleep(0.5)

        try:
            start_time = time.time()

            robot.driveRobot(distance=10, speed=15, sensors=[sensorLeft, sensorCenter])

            sleep(0.5)

            robot.turnRobot(degrees=90, speed=5, direction=left)

            sleep(0.5)

            robot.driveRobot(distance=53, speed=15, sensors=[sensorLeft, sensorCenter, sensorRight])

            sleep(0.5)

            robot.turnRobot(degrees=45, speed=5, direction=left)

            sleep(0.5)

            # Before pipe approaching
            robot.driveRobot(distance=11, speed=5, sensors=[])

            sleep(0.5)

            robot.turnRobot(degrees=50, speed=5, direction=left)

            sleep(0.5)

            extra.motorsOn()

            robot.turnRobot(degrees=5, speed=5, direction=right)

            sleep(0.5)

            robot.driveRobot(distance=2, speed=2, sensors=[])

            sleep(2)

            extra.valveRelease()

            sleep(4)

            extra.motorsOff()

        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")

            extra.motorsOff()

            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            GPIO.cleanup()

    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

