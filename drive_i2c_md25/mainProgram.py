from robotFunctions import *


if __name__ == '__main__':
    try:

        turnRobot(360, 50)
        time.sleep(0.5)
        turnRobot(360, 50, False)
        time.sleep(0.5)
        # turnRobot(360, 10)
        # time.sleep(0.5)
        # turnRobot(360, 10)

    except KeyboardInterrupt:
        print("Stopped by user")
        GPIO.cleanup()

