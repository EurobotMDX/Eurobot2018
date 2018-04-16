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
import thread

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

        sleepAfterEachOperation = 0.6

        while not startSwitch():
            pass

        sleep(sleepAfterEachOperation)

        # Create two threads as follows
        try:
            thread.start_new_thread(extra.timer, (robot,))
            log.info("Start timer thread")

        except Exception as error:
            log.error("Error: unable to start thread %s" %error)

        try:
            start_time = time.time()

            if True:
                robot.driveRobot(distance=10, speed=10, sensors=[sensorLeft, sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=10, direction=left)

                sleep(sleepAfterEachOperation)

                robot.driveRobot(distance=54, speed=15, sensors=[sensorLeft, sensorCenter, sensorRight])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=8, direction=left)

                sleep(sleepAfterEachOperation)

                # Before pipe approaching
                robot.driveRobot(distance=9, speed=5, sensors=[])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=8, direction=left)

                sleep(sleepAfterEachOperation)

                extra.motorsOn()

                sleep(sleepAfterEachOperation)

                # Last straight before pipe approach
                robot.driveRobot(distance=2.7, speed=1, sensors=[])

                sleep(1)

                extra.valveRelease()

                sleep(2)

                extra.motorsOff()

                # Leaving first recuperator
                robot.driveBack(distance=3, speed=2)

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=15, direction=right)

                sleep(1)

                robot.driveRobot(distance=70, speed=20, sensors=[sensorCenter, sensorRight])
                # robot.driveRobot(distance=70, speed=20, sensors=[])

                sleep(sleepAfterEachOperation)

                # Bee deploy
                servoBee.turn(165)

                robot.driveRobot(distance=20, speed=20, sensors=[])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=8, direction=right)

                sleep(sleepAfterEachOperation)

                # Bee close
                servoBee.turn(50)

                robot.turnRobot(degrees=90, speed=15, direction=right)

                sleep(sleepAfterEachOperation)

                # Away from the wall
                robot.driveRobot(distance=10, speed=10, sensors=[sensorLeft, sensorCenter])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=90, speed=15, direction=left)

                sleep(sleepAfterEachOperation)

                # Straight before second pipe
                robot.driveRobot(distance=25, speed=10, sensors=[sensorCenter, sensorRight])

                sleep(sleepAfterEachOperation)

                robot.turnRobot(degrees=45, speed=15, direction=left)

                sleep(sleepAfterEachOperation)

                robot.driveRobot(distance=15, speed=10, sensors=[])

                sleep(sleepAfterEachOperation)

                # Last turn under the pipe
                robot.turnRobot(degrees=58, speed=5, direction=left)

                sleep(sleepAfterEachOperation)

                servoPipe.turn(13)

                sleep(sleepAfterEachOperation)

                servoArm.turn(degrees=86)

                robot.driveRobot(distance=4.5, speed=1, sensors=[])

                sleep(sleepAfterEachOperation + 1)

                extra.valveRelease()

                sleep(1)

                # Do not close the arm
                # servoArm.turn(degrees=160)

        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")

            extra.motorsOff()
            servoArm.turn(160)

            GPIO.cleanup()

        finally:

            elapsed_time = time.time() - start_time

            elapsed_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

            log.info("Finished execution, clean up. Time elapsed: {}".format(elapsed_time))

            extra.motorsOff()

            servoArm.turn(160)

            GPIO.cleanup()



    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)
