from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from settings import logging as log
# from servo import Servo
from time import sleep
import datetime


if __name__ == '__main__':
    canRun = False

    try:
        robot = driving()
        # servo = Servo(15, "Balls pipe servo")
        log.info("Initialized main objects")
        canRun = True
    except Exception as error:
        canRun = False
        log.error("Could not create main objects.!!! Error: %s" % error)

    if robot.checkStatus() and canRun:
        try:
            robot.activateValve()
            # robot.sensorTest()
            # robot.driveBack(30, 10, False)


            # robot.turnRobot(90, 15, False)
            # sleep(0.5)
            # Delay between is used in order to make sure that encoders were reset completely.
            # robot.driveRobot(100, 50)
            # robot.turnRobot(180, 15, False)
            # sleep(0.5)
            # sleep(10)



            # servo.setAngle(1)
            # sleep(1)
            # servo.setAngle(0)
            # sleep(3)
            # servo.setAngle(8)
            # sleep(3)
            # servo.setAngle(0)




            # Servo on 7 stops, middle point

            # servo.setAngle(6.5)

            servo.cleanup()

        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")
            GPIO.cleanup()
    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

