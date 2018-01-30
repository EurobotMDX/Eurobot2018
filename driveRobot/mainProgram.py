from robotFunctions import *
from terminalColors import bcolors as tc

if __name__ == '__main__':
    if checkStatus():
        try:
            # turnRobot(360, 50)
            # time.sleep(0.5)
            driveRobot(400, 40)
            # turnRobot(180, 20, False)
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

