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

        sensorLeft = Sensor(0x72, "left")
        sensorCenter = Sensor(0x71, "centre")
        sensorRight = Sensor(0x70, "right")

        sensors = [sensorLeft, sensorCenter, sensorRight]

        robot.changeAcc(1)

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

        log.debug("Side selection switch value: %s" % sideSwitch())

        log.debug("Expecting for start switch")

        while not startSwitch():
            pass

        if sideSwitch() == "Orange":
            sideSelected = "Orange"

        else:
            sideSelected = "Green"

        if sideSelected == "Orange":
            log.info("Setting up init positions for 'Orange' site")

            servoPipe.turn(14)
            servoBee.turn(165)

        else:
            log.info("Setting up init positions for 'Green' site")

            servoPipe.turn(125)
            servoBee.turn(165)

        # robot.sensorTest([sensorCenter, sensorLeft], 50)

        sleep(1)

        # Create two threads as follows
        try:
            thread.start_new_thread(extra.timer, (robot,))
            log.info("Start timer thread")

        except Exception as error:
            log.error("Error: unable to start thread %s" % error)

        try:
            start_time = time.time()

            if sideSelected == "Orange":

                robot.driveRobot(distance=10, speed=10, sensors=[sensorLeft, sensorCenter], deceleration_offset=0)

                robot.turnRobot(degrees=90, speed=15, direction=left, deceleration_offset=10)

                # first table from a door
                robot.driveRobot(distance=53, speed=20, sensors=[sensorLeft, sensorCenter], deceleration_offset=10)

                robot.turnRobot(degrees=45, speed=10, direction=left, deceleration_offset=10)

                robot.driveRobot(distance=9, speed=10, sensors=[], deceleration_offset=0)

                robot.turnRobot(degrees=45, speed=10, direction=left, deceleration_offset=5)

                extra.motorsOn()

                sleep(1)

                # Last straight before pipe approach
                robot.driveRobot(distance=1, speed=2, sensors=[], deceleration_offset=0)

                sleep(1)

                extra.valveRelease()

                sleep(2)

                extra.valveRelease()

                sleep(2)

                extra.motorsOff()

                # Leaving first recuperator
                robot.driveBack(distance=3, speed=2)

                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=55, speed=20, sensors=[sensorCenter, sensorLeft], deceleration_offset=10)

                servoBee.turn(0)

                robot.driveRobot(distance=35, speed=15, sensors=[], deceleration_offset=10)

                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=5)

                robot.driveRobot(distance=20, speed=20, sensors=[], deceleration_offset=10)

                robot.driveBack(distance=30, speed=10)

                servoBee.turn(165)

            else:

                '''

                robot.driveRobot(distance=15, speed=10, sensors=[sensorLeft, sensorCenter], deceleration_offset=0)

                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=70, speed=20, sensors=[sensorLeft, sensorCenter], deceleration_offset=10)

                robot.turnRobot(degrees=45, speed=10, direction=left, deceleration_offset=10)

                # Before pipe approaching
                robot.driveBack(distance=10, speed=5)

                # robot.driveRobot(distance=9, speed=10, sensors=[], deceleration_offset=0)
                robot.turnRobot(degrees=45, speed=10, direction=right, deceleration_offset=5)

                extra.motorsOn()

                sleep(1)

                # Last straight before pipe approach
                robot.driveBack(distance=2, speed=2, sensors=[], deceleration_offset=0)

                sleep(1)

                extra.valveRelease()

                sleep(2)

                extra.valveRelease()

                sleep(2)

                extra.motorsOff()

                # Leaving first recuperator
                robot.driveRobot(distance=3, speed=5, sensors=[sensorLeft, sensorCenter], deceleration_offset=0)

                # robot.driveRobot(distance=3, speed=2)
                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=55, speed=20, sensors=[sensorCenter, sensorLeft], deceleration_offset=10)

                robot.driveRobot(distance=35, speed=15, sensors=[], deceleration_offset=10)

                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=5, speed=20, sensors=[], deceleration_offset=10)

                robot.turnRobot(degrees=45, speed=15, direction=right, deceleration_offset=5)

                servoBee.turn(0)

                sleep(1)

                robot.turnRobot(degrees=45, speed=15, direction=left, deceleration_offset=5)

                sleep(1)

                robot.driveBack(distance=25, speed=10)

                robot.driveRobot(distance=20, speed=20, sensors=[], deceleration_offset=10)



                '''


                robot.driveRobot(distance=15, speed=10, sensors=[sensorLeft, sensorCenter], deceleration_offset=0)

                # old
                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=160, speed=20, sensors=[], deceleration_offset=10)

                robot.turnRobot(degrees=90, speed=15, direction=right, deceleration_offset=10)

                robot.driveRobot(distance=10, speed=20, sensors=[], deceleration_offset=10)

                robot.turnRobot(degrees=45, speed=15, direction=right, deceleration_offset=5)

                servoBee.turn(0)

                sleep(2)

                robot.turnRobot(degrees=45, speed=15, direction=left, deceleration_offset=5)

                sleep(2)

                robot.driveBack(distance=30, speed=10)

                robot.driveRobot(distance=30, speed=20, sensors=[], deceleration_offset=10)

                servoBee.turn(165)



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
