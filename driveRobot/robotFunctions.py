import time
import RPi.GPIO as GPIO
import sys

sys.path.insert(0, '..')
import robotConfig as config
from terminalColors import bcolors as tc
from sensor import *

dummy = True

try:
    import smbus
    dummy = False
    print('SMBUS is available')
except:
    print('SMBUS not available; in dummy mode')


MD25_DEFAULT_ADDRESS = 0x58
MD25_DEFAULT_MODE = 0

MD25_REGISTER_SPEED1 = 0x00
MD25_REGISTER_SPEED2_TURN = 0x01
MD25_REGISTER_ENC1A = 0x02
MD25_REGISTER_ENC1B = 0x03
MD25_REGISTER_ENC1C = 0x04
MD25_REGISTER_ENC1D = 0x05
MD25_REGISTER_ENC2A = 0x06
MD25_REGISTER_ENC2B = 0x07
MD25_REGISTER_ENC2C = 0x08
MD25_REGISTER_ENC2D = 0x09
MD25_REGISTER_BATTERY_VOLTS = 0x0A
MD25_REGISTER_MOTOR1_CURRENT = 0x0B
MD25_REGISTER_MOTOR2_CURRENT = 0x0C
MD25_REGISTER_SOFTWARE_REV = 0x0D
MD25_REGISTER_ACCELERATION_RATE = 0x0E
MD25_REGISTER_MODE = 0x0F
MD25_REGISTER_COMMAND = 0x10


class md25:
    def __init__(self, mode=MD25_DEFAULT_MODE, bus=1, address=MD25_DEFAULT_ADDRESS):
        self.mode = mode
        self.address = address
        self.bus = None
        print('Dummy is:', dummy)
        if not dummy:
            print('Setting up SMBus\n')
            self.bus = smbus.SMBus(bus)
            self.bus.write_byte_data(self.address, MD25_REGISTER_MODE, self.mode)

    def ensureSet(self, args, message='', all=True):
        for name in args:
            if None == args[name]:
                if all:
                    raise ValueError("%s was not set. %s" % (name, message));
            else:
                return
        raise ValueError("one of %s should be set. %s" % (args.keys(), message));

    def ensureRange(self, range, args, message=''):
        for name in args:
            if args[name] and (args[name] < range[0] or args[name] > range[1]):
                raise ValueError(
                    "%s (%i) was out of range (%i - %i). %s" % (name, args[name], range[0], range[1], message))

    def drive(self, motor0=None, motor1=None, speed=None, turn=None):
        # print (motor0, motor1, speed, turn)
        if 0 == self.mode:
            self.ensureSet({'motor0': motor0, 'motor1': motor1}, all=False)
            self.ensureRange((1, 255), {'motor0': motor0, 'motor1': motor1})
        if 1 == self.mode:
            self.ensureSet({'motor0': motor0, 'motor1': motor1}, all=False)
            self.ensureRange((-128, 127), {'motor0': motor0, 'motor1': motor1})
        if (0 == self.mode or 1 == self.mode) and self.bus:
            if motor0:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, motor0)
            if motor1:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, motor1)
        if 2 == self.mode:
            self.ensureSet({'speed': speed, 'turn': turn}, all=False)
            self.ensureRange((1, 255), {'speed': speed, 'turn': turn})
        if 3 == self.mode:
            self.ensureSet({'speed': speed, 'turn': turn}, all=False)
            self.ensureRange((-128, 127), {'speed': speed, 'turn': turn})
        if (2 == self.mode or 3 == self.mode) and self.bus:
            if speed:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, speed)
                # time.sleep(5)
            if turn:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, turn)
                # time.sleep(5)

    def stop(self):
        if (0 == self.mode or 2 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 128)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 128)
        if (1 == self.mode or 3 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 0)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 0)
            # print('STOP!!!')

    def battery(self):
        if self.bus:
            return self.bus.read_byte_data(self.address, MD25_REGISTER_BATTERY_VOLTS)
        else:
            return 999

    def read_encoder1(self):
        if self.bus:
            try:
                e1 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1A)
                e2 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1B)
                e3 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1C)
                e4 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1D)

                # Return an array of encoder values
                # return [e1, e2, e3, e4]
                totalEncoder = e4 + (255 * e3) + (65025 * e2) + (16581375 * e1)
            except IOError:
                totalEncoder = 0
                print "CAUGHT: IOError"

            return totalEncoder
        else:
            return "Error while reading encoder for motor 1"

    def read_encoder2(self):
        if self.bus:
            try:
                e1 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2A)
                e2 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2B)
                e3 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2C)
                e4 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2D)

                # Return an array of encoder values
                # return [e1, e2, e3, e4]
                totalEncoder = e4 + (255 * e3) + (65025 * e2) + (16581375 * e1)
            except IOError:
                totalEncoder = 0
                print "CAUGHT: IOError"

            return totalEncoder
        else:
            return "Error while reading encoder for motor 1"

    def reset_encoders(self):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x20)
            print("Encoders were reset")
        else:
            print("Could not reset encoders")

    def disable_2s_timeout(self):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x32)
            print("Disabled 2s timeout")
        else:
            print("Error when attempting to disable 2s timeout")

    def enable_2s_timeout(self):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x33)
            print("Disabled 2s timeout")
        else:
            print("Error when attempting to disable 2s timeout")

    def setAcceleration(self, value):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_ACCELERATION_RATE, value)
            print("Changed acceleration mode")
        else:
            print("Error when acceleration")


# Init configuration for robot
mainRobot = md25(mode=1)

circumferenceOfCircle = config.robotSettings['circumferenceOfCircle']
oneEncMM = config.robotSettings['oneEncMM']
sensorThreshold = config.robotSettings['sensorThreshold']

# Setup sensors
sensor1 = Sensor(11, "Front", 0.02)


def showCounterForWheel(timein=10):
    '''
    This function prints encoders values for given time in variable 'timein'.
    :param timein:
    :return:
    '''
    countdown = time.time() + timein
    startTime = time.time()

    print("Countdown is: {} startTime is: {}".format(countdown, startTime))

    mainRobot.reset_encoders()

    while (countdown > startTime):
        print("Encoders values are --- encoder 1: {} --- encoder 2: {}\n".format(mainRobot.read_encoder1(),
                                                                                 mainRobot.read_encoder2()))

def travelledDistance(distance, current):
    '''
    Function takes two inputs such as; current distance travelled and destination distance, calculates the percentages of travelled distance
    :param distance: float
    :param current: float
    :return: percentage of travelled distance
    '''
    if current != 0:
        remainingDistancePercentage = (current / distance) * 100
        return round(remainingDistancePercentage, 1)
    else:
        return 0


def driveRobot(distance, speed):
    '''
    This function drives a robot forward. By adjusting values such as: speed and distance can control a robot.
    :param distance:
    :param speed:
    :return:
    '''
    mainRobot.reset_encoders()

    encoderDestination = distance / oneEncMM

    sensor1.setUp()

    encoder1Reading = mainRobot.read_encoder1()
    encoder2Reading = mainRobot.read_encoder2()

    distance = float(distance)

    # Change acceleration mode if necessary
    # changeAcc(10)

    while (mainRobot.read_encoder1() <= encoderDestination and mainRobot.read_encoder2() <= encoderDestination):
        # Check if sensor detected any obstacle on the way if yes then stop the robot and wait
        if sensor1.getSensorValue() <= sensorThreshold:
            mainRobot.stop()
        else:
            mainRobot.drive(speed, speed)

        encodersAvg = (encoder1Reading + encoder1Reading) / 2.0

        currentTravelDistance = round(encodersAvg * oneEncMM, 3)

        tDist = travelledDistance(distance, currentTravelDistance)

        print ("Travelled distance: {}".format(tDist))

        # if (tDist > 85):
        #     speed = 10

        encoder1Reading = mainRobot.read_encoder1()
        encoder2Reading = mainRobot.read_encoder2()

    else:
        sensor1.stopSensor()
        mainRobot.stop()


def turnRobot(degrees, speed, clockwise=True):
    '''
    This function turns a robot. Depending on the argument 'clockwise', a robot can turn right or left
    :param degrees:
    :param speed:
    :param clockwise:
    :return:
    '''
    mainRobot.reset_encoders()

    oneWheelDistance = (circumferenceOfCircle / 360) * degrees

    encoderCount = oneWheelDistance / oneEncMM

    if clockwise:
        while mainRobot.read_encoder1() <= encoderCount:
            mainRobot.drive(speed, -speed)
        else:
            mainRobot.stop()

    elif not clockwise:
        while mainRobot.read_encoder2() <= encoderCount:
            mainRobot.drive(-speed, speed)
        else:
            mainRobot.stop()

    else:
        print("Error while robot turning!")


def sensorTest(timein=10):
    '''
    This function prints sensor values for given time in variable 'timein'.
    :param timein:
    :return:
    '''
    countdown = time.time() + timein
    startTime = time.time()

    sensor1.setUp()

    while (countdown > startTime):
        print (sensor1.getSensorValue)

    else:
        print("Stopped")


def checkStatus():
    canRun = True

    getBatteryVoltage = mainRobot.battery()
    getBatteryVoltage = getBatteryVoltage / 10.0

    print("\n" + tc.FAIL + "Battery Status: " + str(getBatteryVoltage) + "V" + tc.ENDC + "\n")

    if float(getBatteryVoltage) < 11.0:
        print(tc.FAIL + "Critical Battery Level. PLEASE REPLACE BATTERY!" + tc.ENDC)
        canRun = False

    else:
        print(tc.OKGREEN + "Battery in good level." + tc.ENDC + "\n")
        canRun = True

    return canRun

def changeAcc(value=5):
    print("Changed acceleration to: {}".format(value))
    mainRobot.setAcceleration(value)

