from robotFunctions import *
from terminalColors import bcolors as tc
import sys
sys.path.insert(0, '..')
from settings import logging as log

if __name__ == '__main__':

    robot = driving()

    if robot.checkStatus():
        try:
            # Delay between is used in order to make sure that encoders were reset completely.
            # driveRobot(100, 50)
            robot.turnRobot(180, 20, False)
            # time.sleep(1)
            # robot.driveRobot(178, 120)
            # time.sleep(1)
            # robot.turnRobot(180, 60, False)
            # robot.driveRobot(178, 120)
            # time.sleep(1)
            # robot.turnRobot(180, 60, False)

            # sensorTest()

        except KeyboardInterrupt:
            log.debug("\nStopped by user\n")
            GPIO.cleanup()
    else:
        log.info(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

