from robotFunctions import *
from terminalColors import bcolors as tc

if __name__ == '__main__':

    robot = driving()

    if robot.checkStatus():
        try:
            # Delay between is used in order to make sure that encoders were reset completely.
            # driveRobot(100, 50)
            robot.turnRobot(180, 10, False)
            time.sleep(1)

            robot.driveRobot(178, 120)
            # driveRobot(178, 40)
            # time.sleep(3)
            # driveRobot(178, 50)
            # time.sleep(4)
            # turnRobot(180, 3, False)



            # driveRobot(100, 127)

            # time.sleep(2)
            # changeAcc(10)
            # time.sleep(2)
            # driveRobot(1000, 127)
            # changeAcc(1)
            # time.sleep(2)
            # driveRobot(1000, 127)


            # sensorTest()

        except KeyboardInterrupt:
            print("\nStopped by user\n")
            GPIO.cleanup()
    else:
        print(tc.FAIL + tc.UNDERLINE + "Could not start the program please check the conditions of the robot!" + tc.ENDC)

