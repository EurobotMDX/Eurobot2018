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
    # if robot.checkStatus() and canRun:
    if canRun:
        if sideSwitch() == "Orange":
            log.info("Set 'Orange servo init, position: 8")

            servoPipe.turn(8)
        else:
            log.info("Set 'Orange servo init, position: 8")

            servoPipe.turn(8)

        log.debug("Expecting for start switch")

        log.debug("Side selection switch value: %s" % sideSwitch())

        while not startSwitch():
            pass

        sleep(0.5)

        try:

            turnSpeed = 15

            start_time = time.time()

            # robot.driveRobot(200, 30, sensors)

            servoPipe.turn(98)
            sleep(2)
            servoPipe.turn(8)
            sleep(2)

            # robot.turnRobot(90, turnSpeed, True)

            # robot.sensorTest(sensors, 60)

            # servoPipe.turn(12)
            # sleep(2)
            # servoPipe.turn(8)


            # for i in range(0, 0):
            #
            #     robot.turnRobot(90, turnSpeed, False)
            #     sleep(0.3)


            # distance, speed, sensorEnabled=True, centerSensorOn=True, rightSensorOn=True, leftSensorOn=True

            # robot.driveRobot(20, 20, False, True, True, True)
            # robot.driveRobot(20, 20, False, False, False, False)

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

