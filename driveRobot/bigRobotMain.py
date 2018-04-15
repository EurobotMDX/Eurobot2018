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
            servoBee.turn(50)
            servoArm.turn(160)

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

            if True:
                robot.driveRobot(distance=10, speed=15, sensors=[sensorLeft, sensorCenter])

                sleep(0.5)

                robot.turnRobot(degrees=90, speed=5, direction=left)

                sleep(1)

                robot.driveRobot(distance=52, speed=20, sensors=[sensorLeft, sensorCenter, sensorRight])

                sleep(0.5)

                robot.turnRobot(degrees=45, speed=5, direction=left)

                sleep(0.5)

                # Before pipe approaching
                robot.driveRobot(distance=11, speed=5, sensors=[])

                sleep(0.5)

                robot.turnRobot(degrees=45, speed=3, direction=left)

                sleep(0.5)

                extra.motorsOn()

                sleep(0.5)

                robot.driveRobot(distance=3, speed=1, sensors=[])

                sleep(2)

                extra.valveRelease()

                sleep(4)

                extra.motorsOff()

                # Leaving First recuperator
                robot.driveBack(distance=3, speed=2)
                sleep(1)

                robot.turnRobot(degrees=90, speed=15, direction=right)

                sleep(1)

                robot.driveRobot(distance=70, speed=20, sensors=[sensorCenter, sensorRight])

                sleep(0.5)

                # Bee deploy
                servoBee.turn(165)

                robot.driveRobot(distance=20, speed=20, sensors=[])

                sleep(0.5)

                robot.turnRobot(degrees=90, speed=8, direction=right)

                sleep(1)

                # Bee close
                servoBee.turn(50)

                robot.turnRobot(degrees=90, speed=15, direction=right)

            # if True:
                sleep(0.5)

                robot.driveRobot(distance=10, speed=10, sensors=[sensorLeft, sensorCenter, sensorRight])

                sleep(0.5)

                robot.turnRobot(degrees=90, speed=15, direction=left)

                sleep(0.5)

                robot.driveRobot(distance=25, speed=10, sensors=[sensorCenter, sensorRight])

                sleep(0.5)

                robot.turnRobot(degrees=45, speed=15, direction=left)

                sleep(0.5)

                robot.driveRobot(distance=14, speed=10, sensors=[])

                sleep(0.5)

                robot.turnRobot(degrees=55, speed=4, direction=left)

                sleep(0.5)

                servoPipe.turn(13)

                sleep(1)

                servoArm.turn(degrees=85)

                sleep(0.5)

                extra.valveRelease()

                sleep(5)

                servoArm.turn(degrees=160)


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
