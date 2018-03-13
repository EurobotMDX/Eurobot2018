import time
import RPi.GPIO as GPIO
import sys

sys.path.insert(0, '..')
import settings as config
from terminalColors import bcolors as tc
from sensor import *

log = config.logging

dummy = True

try:
    import smbus

    dummy = False
    log.info('SMBUS is available')
except:
    log.info('SMBUS not available; in dummy mode')

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
        log.info('Dummy is: %s' % dummy)

        if not dummy:
            log.debug('Setting up SMBus')
            try:
                self.bus = smbus.SMBus(bus)
                self.bus.write_byte_data(self.address, MD25_REGISTER_MODE, self.mode)
            except IOError:
                log.error("IO error. Please check if batter is connected and MD25 working correctly.")

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
        try:
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
                if turn:
                    self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, turn)
        except IOError:
            log.warrning("CAUGHT: IOError")

    def stop(self):
        if (0 == self.mode or 2 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 128)
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 128)
            except IOError:
                log.warrning("CAUGHT: IOError")

        if (1 == self.mode or 3 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 0)
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 0)
            except IOError:
                log.warrning("CAUGHT: IOError")

    def battery(self):
        if self.bus:
            return self.bus.read_byte_data(self.address, MD25_REGISTER_BATTERY_VOLTS)
        else:
            log.error("Problem while reading the batter voltage!")
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
                log.warrning("CAUGHT: IOError")

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
                print("CAUGHT: IOError")

            return totalEncoder
        else:
            return "Error while reading encoder for motor 1"

    def reset_encoders(self):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x20)
            print("Encoders are reset")
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


class driving():
    def __init__(self):
        self.mainRobot = md25(mode=1)

        # Init configuration for robot
        self.circumferenceOfCircle = config.robotSettings['circumferenceOfCircle']
        self.oneEncMM = config.robotSettings['oneEncMM']
        self.sensorThreshold = config.robotSettings['sensorThreshold']

        # Setup sensors
        # self.sensor1 = Sensor(33, "Right", 0.02)
        self.sensor1 = Sensor(35, "Middle", 0.02)
        # self.sensor2 = Sensor(37, "Left", 0.02)

        # Slowing down variables
        self.enterSpeedLoop1 = True
        self.enterSpeedLoop2 = False

        # Setup Valve
        self.valvePin = 40
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.valvePin, GPIO.OUT)
        GPIO.output(self.valvePin, GPIO.LOW)

    def setValve(self, state=True):
        if (state):
            GPIO.output(self.valvePin, GPIO.HIGH)
        else:
            GPIO.output(self.valvePin, GPIO.LOW)

    def showCounterForWheel(self, timein=10):
        '''
        This function prints encoders values for given time in variable 'timein'.
        :param timein:
        :return:
        '''
        countdown = time.time() + timein
        startTime = time.time()

        print("Countdown is: {} startTime is: {}".format(countdown, startTime))

        self.self.mainRobot.reset_encoders()

        while (countdown > startTime):
            print("Encoders values are --- encoder 1: {} --- encoder 2: {}\n".format(self.self.mainRobot.read_encoder1(),
                                                                               self.mainRobot.read_encoder2()))

    def travelledDistance(self, distance, current):
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

    def calcStoppingThreshold(self, speed):
        enterSecondSlowDownOnly = False

        if speed < 15:
            enterSecondSlowDownOnly = True

        if (speed >= 110):
            threshold1 = 70.0
            threshold2 = 95.0

        elif (speed >= 90):
            threshold1 = 75.0
            threshold2 = 95.0

        elif (speed >= 70):
            threshold1 = 80.0
            threshold2 = 95.0
        else:
            threshold1 = 85.0
            threshold2 = 95.0

        return [threshold1, threshold2, enterSecondSlowDownOnly]

    def speedControl(self, speed, tDist, stoppingThresholds, turn=False):
        speedThreshold = 3

        if turn:
            speedThreshold = 2

        if stoppingThresholds[2] == True:
            self.enterSpeedLoop1 = False
            self.enterSpeedLoop2 = True

        if self.enterSpeedLoop1 and tDist > stoppingThresholds[0] and speed > 11 and not stoppingThresholds[2]:

            speed = speed - 1

            if speed > 30:
                speed = 29

            if speed == 12:
                self.enterSpeedLoop1 = False
                self.enterSpeedLoop2 = False
                print("Slow down 1 finished. Speed = {}".format(speed))

        if self.enterSpeedLoop2 and tDist > stoppingThresholds[1] and speed >= speedThreshold:

            speed = speed - 1

            if speed == speedThreshold:
                print("Slow down 2 finished. Speed = {}".format(speed))
                self.speedAdjusted2 = True

        return speed

    def driveRobot(self, distance, speed):
        '''
        This function drives a robot forward. By adjusting values such as: speed and distance can control a robot.
        :param distance:
        :param speed:
        :return:
        '''
        self.mainRobot.reset_encoders()

        encoderDestination = distance / self.oneEncMM

        self.sensor1.setUp()

        encoder1Reading = self.mainRobot.read_encoder1()
        encoder2Reading = self.mainRobot.read_encoder2()

        distance = float(distance)

        stoppingThresholds = self.calcStoppingThreshold(speed)

        # Change acceleration mode if necessary
        # changeAcc(10)

        while encoder1Reading <= encoderDestination and encoder2Reading <= encoderDestination:
            # Check if sensor detected any obstacle on the way if yes then stop the robot and wait
            if self.sensor1.getSensorValue() <= self.sensorThreshold:
                self.mainRobot.stop()
            else:
                self.mainRobot.drive(speed, speed)

            encodersAvg = (encoder1Reading + encoder1Reading) / 2.0

            currentTravelDistance = round(encodersAvg * self.oneEncMM, 3)

            tDist = self.travelledDistance(distance, currentTravelDistance)

            print("Travelled distance: {}".format(tDist))

            # Enter this function to slow down
            speed = self.speedControl(speed, tDist, stoppingThresholds)

            # print("Speed is {}".format(speed))

            encoder1Reading = self.mainRobot.read_encoder1()
            encoder2Reading = self.mainRobot.read_encoder2()

        else:
            self.sensor1.stopSensor()
            self.mainRobot.stop()

    def turnRobot(self, degrees, speed, clockwise=True):
        '''
        This function turns a robot. Depending on the argument 'clockwise', a robot can turn right or left
        :param degrees:
        :param speed:
        :param clockwise:
        :return:
        '''
        self.mainRobot.reset_encoders()

        oneWheelDistance = (self.circumferenceOfCircle / 360) * degrees

        encoderDestination = oneWheelDistance / self.oneEncMM

        stoppingThresholds = self.calcStoppingThreshold(speed)

        encoder1Reading = self.mainRobot.read_encoder1()
        encoder2Reading = self.mainRobot.read_encoder2()

        if clockwise:
            while encoder1Reading <= encoderDestination:
                self.mainRobot.drive(speed, -speed)

                tDist = self.travelledDistance(encoderDestination, encoder1Reading)

                speed = self.speedControl(speed, tDist, stoppingThresholds, True)

                print("Travelled distance: {}".format(tDist))

                encoder1Reading = self.mainRobot.read_encoder1()

            else:
                self.mainRobot.stop()

        elif not clockwise:

            while encoder2Reading <= encoderDestination:

                self.mainRobot.drive(-speed, speed)

                tDist = self.travelledDistance(encoderDestination, encoder2Reading)

                speed = self.speedControl(speed, tDist, stoppingThresholds, True)

                print("Travelled distance: {}".format(tDist))

                encoder2Reading = self.mainRobot.read_encoder2()

            else:
                self.mainRobot.stop()

        else:
            print("Error while robot turning the robot!")

    def driveBack(self, distance, speed, sensorEnabled=True):
        '''
        This function drives a robot backward. By adjusting values such as: speed and distance can control a robot.
        :param distance:
        :param speed:
        :return:
        '''
        self.mainRobot.reset_encoders()

        encoderDestination = distance / self.oneEncMM

        self.sensor1.setUp()

        encoder1Reading = self.mainRobot.read_encoder1()
        encoder2Reading = self.mainRobot.read_encoder2()

        distance = float(distance)

        # stoppingThresholds = self.calcStoppingThreshold(speed)

        # Change acceleration mode if necessary
        # changeAcc(10)

        while encoder1Reading >= encoderDestination and encoder2Reading >= encoderDestination or encoder1Reading == 0 or encoder2Reading == 0:

            encodersAvg = (encoder1Reading + encoder1Reading) / 2.0
            currentTravelDistance = round(encodersAvg * self.oneEncMM, 3)
            tDist = self.travelledDistance(distance, currentTravelDistance)

            # Check if sensor detected any obstacle on the way if yes then stop the robot and wait
            if sensorEnabled == True:
                if self.sensor1.getSensorValue() <= self.sensorThreshold:
                    print("Obstacle detected by {}".format(self.sensor1.sensorPosition))
                    self.mainRobot.stop()
                else:
                    self.mainRobot.drive(-speed, -speed)
                    print("Travelled distance: {}".format(tDist))
            else:
                print("Travelled distance: {}  {}".format(tDist, encodersAvg))


            # Enter this function to slow down
            # speed = self.speedControl(speed, tDist, stoppingThresholds)

            # print("Speed is {}".format(speed))

            encoder1Reading = self.mainRobot.read_encoder1()
            encoder2Reading = self.mainRobot.read_encoder2()

        else:
            self.sensor1.stopSensor()
            self.mainRobot.stop()

    def sensorTest(self, timein=10):
        '''
        This function prints sensor values for given time in variable 'timein'.
        :param timein:
        :return:
        '''
        countdown = time.time() + timein

        endTime = time.time()

        self.sensor1.setUp()



        while (countdown > endTime):

            # sensors = [self.sensor1.getSensorValue(), self.sensor2.getSensorValue(), self.sensor1.getSensorValue()]

            # print("Sensor1: {} | Sensor2: {} | Sensor3: {}".format(sensors[0], sensors[1], sensors[2]))

            print(self.sensor1.getSensorValue())

            endTime = time.time()
        else:
            print("Stopped")

    def checkStatus(self):
        canRun = True

        getBatteryVoltage = self.mainRobot.battery()
        getBatteryVoltage = getBatteryVoltage / 10.0

        print("\n" + tc.FAIL + "Battery Status: " + str(getBatteryVoltage) + "V" + tc.ENDC + "\n")

        if float(getBatteryVoltage) < 11.0:
            log.error(tc.FAIL + "Critical Battery Level. PLEASE REPLACE BATTERY!" + tc.ENDC)
            canRun = False

        else:
            log.info(tc.OKGREEN + "Battery in good level." + tc.ENDC + "\n")

        return canRun

    def changeAcc(self, value=5):
        log.info("Changed acceleration to: {}".format(value))
        self.mainRobot.setAcceleration(value)