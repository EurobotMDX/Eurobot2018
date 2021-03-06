import sys
sys.path.insert(0, '..')
from time import sleep
import time
import datetime
import thread

from robotFunctions import *
from terminalColors import bcolors as tc
from servoControl import servoControl
from Switches import *
from Sensor import *
from settings import logging as log
import settings as config

if __name__ == '__main__':
    canRun = False

    sensorCenter = None
    sensorRight = None
    sensorLeft = None

    right = True
    left = False
    turnSpeed = 15

    servoPipeChannel = config.robotSettings['servoPipeChannel']
    servoBeeChannel = config.robotSettings['servoBeeChannel']

    try:
        robot = Driving()
        # Initialize servos

        servoPipe = servoControl("Pipe servo", servoPipeChannel, 60)
        servoBee = servoControl("Bee servo", servoBeeChannel, 60)
        # servoArm = servoControl("Arm servo", 1, 60)

        sensorCenter = Sensor(0x72, "centre")
        sensorRight = Sensor(0x71, "right")
        sensorLeft = Sensor(0x73, "left")

        sensors = [sensorLeft, sensorCenter, sensorRight]

        extra = RobotHelpers()

        log.info("Initialized main objects")

        if robot.checkStatus():
            canRun = True
        else:
            canRun = False

    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if canRun:

        while not startSwitch():
            pass

        if sideSwitch() == "Orange":

            sideSelected = "Orange"

        else:
            sideSelected = "Green"

        if sideSelected == "Orange":
            log.info("Setting up init positions for 'Orange' site")

            servoPipe.turn(125) # 125 Initial, 20 Deployed
            servoBee.turn(11) # 11 Initial, 180 Deployed
            # servoArm.turn(165) # 165 Initial, 100 Deployed

        else:
            log.info("Setting up init positions for 'Green' site")

            servoPipe.turn(20)
            servoBee.turn(11)
            # servoArm.turn(165)

        log.debug("Side selection switch value: %s" % sideSwitch())

        log.debug("Expecting for start switch")

        sleepAfterEachOperation = 1.1

        sleep(sleepAfterEachOperation)

        # Create two threads as follows
        try:
            thread.start_new_thread(extra.timer, (robot,))
            log.info("Start timer thread")

        except Exception as error:
            log.error("Error: unable to start thread %s" %error)

        try:
            start_time = time.time()

            if sideSelected == "Orange":

                sleep(2)

                robot.driveRobot(distance=10, speed=5, sensors=[sensorLeft, sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=3, direction=left)

                sleep(sleepAfterEachOperation)

                # first table from a door
                # robot.driveRobot(distance=55, speed=15, sensors=[sensorLeft, sensorCenter, sensorRight])
                # Differenence in tables
                # Second table
                robot.driveRobot(distance=54, speed=15, sensors=[sensorLeft, sensorCenter, sensorRight])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=5, direction=left)

                sleep(sleepAfterEachOperation)

                # Before pipe approaching
                # first table
                # robot.driveRobot(distance=9, speed=2, sensors=[])
                # second table
                robot.driveRobot(distance=8, speed=2, sensors=[])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=10, direction=left)

                sleep(sleepAfterEachOperation)

                extra.motorsOn()

                sleep(sleepAfterEachOperation)

                # Last straight before pipe approach
                # First
                # robot.driveRobot(distance=3, speed=1, sensors=[])
                # Second
                robot.driveRobot(distance=5, speed=1, sensors=[])

                sleep(1)

                extra.valveRelease()

                sleep(2)

                extra.valveRelease()

                sleep(2)

                extra.motorsOff()

                # Leaving first recuperator
                robot.driveBack(distance=3, speed=2)

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=4, direction=right)

                sleep(sleepAfterEachOperation + 1)

                robot.driveRobot(distance=70, speed=20, sensors=[sensorCenter, sensorRight])

                sleep(sleepAfterEachOperation)

                # Bee deploy
                servoBee.turn(180)

                robot.driveRobot(distance=20, speed=20, sensors=[])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=8, direction=right)

                sleep(sleepAfterEachOperation)

                # Bee close
                servoBee.turn(11)

                robot.turnRobot(degrees=45, speed=10, direction=right)

                sleep(sleepAfterEachOperation)

            else:

                sleep(1)

                robot.driveRobot(distance=13, speed=8, sensors=[sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=7, direction=right)

                sleep(sleepAfterEachOperation)

                robot.driveRobot(distance=67, speed=15, sensors=[sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=8, direction=left)

                sleep(sleepAfterEachOperation)

                # Before pipe approaching
                robot.driveBack(distance=5, speed=2)

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=6, direction=left)

                sleep(sleepAfterEachOperation)

                extra.motorsOn()

                sleep(sleepAfterEachOperation)

                # Last straight before pipe approach
                robot.driveBack(distance=3, speed=1)

                sleep(sleepAfterEachOperation)

                extra.valveRelease()

                sleep(2)

                extra.valveRelease()

                sleep(2)

                extra.motorsOff()

                # Leaving first recuperator
                robot.driveRobot(distance=6, speed=5, sensors=[sensorLeft, sensorCenter, sensorRight])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=85, speed=6, direction=right)

                # sleep(sleepAfterEachOperation)

                # robot.turnRobot(degrees=45, speed=10, direction=right)

                sleep(1)

                robot.driveRobot(distance=60, speed=15, sensors=[sensorLeft, sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.driveRobot(distance=30, speed=10, sensors=[])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=15, direction=right)

                sleep(sleepAfterEachOperation)

                robot.driveRobot(distance=8, speed=10, sensors=[])

                sleep(sleepAfterEachOperation)

                # Bee deploy
                servoBee.turn(180)
                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=225, speed=15, direction=left)

                sleep(sleepAfterEachOperation)

                # Bee close
                servoBee.turn(11)

                sleep(sleepAfterEachOperation)

                robot.driveBack(distance=4, speed=2)

                # robot.turnRobot(degrees=90, speed=15, direction=right)

        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")

            extra.motorsOff()

            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            # extra.motorsOff()

            GPIO.cleanup()

    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)
