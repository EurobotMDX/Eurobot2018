from robotFunctions import *


if __name__ == '__main__':
    try:
        printBatteryVoltage()

        # turnRobot(360, 50)
        # time.sleep(0.5)
        driveRobot(20, 20)
        # turnRobot(180, 50, False)
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
        print("Stopped by user")
        GPIO.cleanup()

