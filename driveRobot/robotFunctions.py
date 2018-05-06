import time
import RPi.GPIO as GPIO
import sys
import thread
sys.path.insert(0, '..')
import settings as config
from terminalColors import bcolors as tc
import math
from sensorSmallRobot import *
from compass import *
from time import sleep

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
MD25_REGISTER_SOFTWARE_REV = 0x0D
MD25_REGISTER_ACCELERATION_RATE = 0x0E
MD25_REGISTER_MODE = 0x0F
MD25_REGISTER_COMMAND = 0x10
MD25_REGISTER_GET_CURRENT_1 = 0x11
MD25_REGISTER_GET_CURRENT_2 = 0x12

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
                    raise ValueError("%s was not set. %s" % (name, message))
            else:
                return
        raise ValueError("one of %s should be set. %s" % (args.keys(), message))

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
            log.warning("CAUGHT: IOError 'drive'")
            self.drive(motor0, motor1, speed, turn)

    def stop1(self):
        if (0 == self.mode or 2 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 128)
            except IOError:
                log.warning("CAUGHT: IOError 'stop motor1'")
                self.stop()

        if (1 == self.mode or 3 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 0)
            except IOError:
                log.warning("CAUGHT: IOError 'stop motor1'")
                self.stop()

    def stop2(self):
        if (0 == self.mode or 2 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 128)
            except IOError:
                log.warning("CAUGHT: IOError 'stop motor2'")
                self.stop()

        if (1 == self.mode or 3 == self.mode) and self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 0)
            except IOError:
                log.warning("CAUGHT: IOError 'stop motor2'")
                self.stop()

    def battery(self):
        if self.bus:
            return self.bus.read_byte_data(self.address, MD25_REGISTER_BATTERY_VOLTS)
        else:
            log.error("Problem while reading the batter voltage!")
            return 999

    def get_current_motor_1(self):
        if self.bus:
            try:
                return self.bus.read_byte_data(self.address, MD25_REGISTER_GET_CURRENT_1)
            except IOError:
                log.warning("CAUGHT: IOError 'get_current_motor_1'")
        else:
            log.error("Problem while reading the current from motor1!")

    def get_current_motor_2(self):
        if self.bus:
            try:
                return self.bus.read_byte_data(self.address, MD25_REGISTER_GET_CURRENT_2)

            except IOError:
                log.warning("CAUGHT: IOError 'get_current_motor_2'")
        else:
            log.error("Problem while reading the current from motor2!")

    def get_speed_motor_1(self):
        if self.bus:
            try:
                return self.bus.read_byte_data(self.address, MD25_REGISTER_SPEED1)

            except IOError:
                log.warning("CAUGHT: IOError 'get_speed_motor_1'")
        else:
            log.error("Problem while reading the current from get_speed_motor_1!")

    def get_speed_motor_2(self):
        if self.bus:
            try:
                return self.bus.read_byte_data(self.address, MD25_REGISTER_SPEED2_TURN)

            except IOError:
                log.warning("CAUGHT: IOError 'get_speed_motor_2'")
        else:
            log.error("Problem while reading the current from get_speed_motor_2!")

    def read_encoder1(self):
        if self.bus:
            totalEncoder = 0
            try:
                e1 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1A)
                e2 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1B)
                e3 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1C)
                e4 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1D)

                # Return an array of encoder values
                # return [e1, e2, e3, e4]
                totalEncoder = e4 + (255 * e3) + (65025 * e2) + (16581375 * e1)
            except IOError:
                log.warning("CAUGHT: IOError 'read_encoder1'")
                self.read_encoder1()

            return totalEncoder
        else:
            return "Error while reading encoder for motor 1"

    def read_encoder2(self):
        if self.bus:
            totalEncoder = 0
            try:
                e1 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2A)
                e2 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2B)
                e3 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2C)
                e4 = self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2D)

                # Return an array of encoder values
                # return [e1, e2, e3, e4]
                totalEncoder = e4 + (255 * e3) + (65025 * e2) + (16581375 * e1)
            except IOError:
                log.warning("CAUGHT: IOError 'read_encoder2'")
                self.read_encoder2()

            return totalEncoder
        else:
            return "Error while reading encoder for motor 1"

    def send_command(self, command):
        try:
            self.bus.read_byte_data(self.address, command)
            log.info("Send command to MD25, command: {}".format(command))

        except IOError:
            log.warning("CAUGHT: IOError send_command")

    def reset_encoders(self):
        if self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x20)
                log.info("Encoders were successfully reset")

            except IOError:
                log.warning("CAUGHT: IOError 'Encoders are reset'")
                self.reset_encoders()

        else:
            log.error("Could not reset encoders")

    def disable_auto_speed_reg(self):
        if self.bus:
            try:
                self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x30)
                log.info("Disabled auto speed regulation")

            except IOError:
                log.warning("CAUGHT: IOError 'disable_auto_speed_reg'")
                self.reset_encoders()

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

    def setAcceleration(self, value):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_ACCELERATION_RATE, value)

class Driving:
    def __init__(self):
        self.mainRobot = md25(mode=1)

        # Init configuration for robot
        self.circumferenceOfCircle = config.robotSettings['circumferenceOfCircle']
        self.oneEncMM = config.robotSettings['oneEncMM']
        self.sensorThreshold = config.robotSettings['sensorThreshold']
        self.encoderMaxValue = config.robotSettings['encoderMaxValue']
        self.robotType = config.robotSettings['robotType']
        self.speed_interval = 0.06
        self.min_deceleration_speed = 1
        self.rest_after_drive = 1
        self.deceleration_offset = 12
        # self.acceleration = 2
        # self.changeAcc(self.acceleration)

        # Enables automatic speed regulation (default)
        # self.mainRobot.send_command(0x31)

        self.when = None

        # Setup Valve
        self.valvePin = 40
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.valvePin, GPIO.OUT)
        GPIO.output(self.valvePin, GPIO.LOW)

    def checkForObstacle(self, sensors, obstacleClear=True):
        """
        This function check the value for sensors and determince if there is any obstacle on the way.
        :param centerSensorOn:
        :param rightSensorOn:
        :param leftSensorOn:
        :return: obstacle - Boolean value
        """
        obstacle = False

        if self.robotType == "main":

            for sensor in sensors:
                sensor.reqDistance()

            if len(sensors) > 0:
                sleep(sensors[0].delay_time)

            for sensor in sensors:
                if sensor.readDistanceResponse() <= self.sensorThreshold:
                    obstacle = True

                    if obstacleClear:
                        log.info("Obstacle for sensor {} distance: {}".format(sensor.position, sensor.lastReading))

            return obstacle

        if self.robotType == "small":
            for sensor in sensors:

                distance = getSensorHCValue(sensor)
                if distance <= self.sensorThreshold:
                    obstacle = True

                    if obstacleClear:
                        log.info("Obstacle for sensor {} distance: {}".format(sensor, distance))

            return obstacle

    def stopDriving(self):
        self.mainRobot.stop1()
        self.mainRobot.stop2()
        log.info("Emergency stop driving")

    def setValve(self, state=True):
        if (state):
            GPIO.output(self.valvePin, GPIO.HIGH)
        else:
            GPIO.output(self.valvePin, GPIO.LOW)

    def showCounterForWheel(self, timein=10):
        """
        This function prints encoders values for given time in variable 'timein'.
        :param timein:
        :return:
        """
        countdown = time.time() + timein
        startTime = time.time()

        print("Countdown is: {} startTime is: {}".format(countdown, startTime))

        self.self.mainRobot.reset_encoders()

        while (countdown > startTime):
            print("Encoders values are --- encoder 1: {} --- encoder 2: {}\n".format(self.self.mainRobot.read_encoder1(), self.mainRobot.read_encoder2()))

    def travelledDistance(self, distance, current):
        """
        Function takes two inputs such as; current distance travelled and destination distance, calculates the percentages of travelled distance
        :param distance: float
        :param current: float
        :return: percentage of travelled distance
        """
        if current != 0:
            remainingDistancePercentage = (current / distance) * 100
            return round(remainingDistancePercentage, 1)
        else:
            return 0

    def deceleration_control(self, speed, step=1):

        if time.time() >= self.when and speed > self.min_deceleration_speed:
            self.when = time.time() + self.speed_interval
            speed -= step
            # print("speed: {}".format(speed))

        return speed

    def getDecelerationDistance(self, distance, speed, offset=10):
        """
        Calculates distance from which the robot will start slowing down
        :param distance:
        :param speed:
        :param offset:
        :return:
        """
        # f value indicates any possible skirt on the surface
        f = 0.9
        return (distance - math.pow(speed, 2) / (250 * f)) - offset

    def driveRobot(self, distance, speed, sensors):
        """
        This function drives a robot forward. By adjusting values such as: speed and distance can control a robot.
        :param distance:
        :param speed:
        :param sensors:
        :return:
        """
        distance = float(distance)

        self.mainRobot.reset_encoders()

        deceleration_threshold = self.getDecelerationDistance(distance, speed, self.deceleration_offset)

        log.info("Drive forward distance: {} | speed: {} | start deceleration at: {}".format(distance, speed, deceleration_threshold))

        obstacleClear = True

        encoderDestination = distance / self.oneEncMM

        motor1Drive = True
        motor2Drive = True

        init_time = False

        while True:

            if self.checkForObstacle(sensors, obstacleClear):

                obstacleClear = False

                self.mainRobot.stop1()
                self.mainRobot.stop2()

            else:
                encoder1Reading = self.mainRobot.read_encoder1()
                encoder2Reading = self.mainRobot.read_encoder2()

                obstacleClear = True

                encodersAvg = int((encoder1Reading + encoder2Reading) / 2.0)

                currentTravelDistance = round(encodersAvg * self.oneEncMM, 3)

                # Control the deceleration distance
                if currentTravelDistance >= deceleration_threshold:
                    if not init_time:
                        self.when = time.time() + self.speed_interval
                        init_time = True
                        speed = self.deceleration_control(speed)

                    else:
                        speed = self.deceleration_control(speed)

                # For debug only
                # travelledDistance = self.travelledDistance(distance, currentTravelDistance)
                # print("Travelled distance: {}".format(travelledDistance))
                # print("Speed {}".format(currentTravelDistance / (time.time() - start_driving_time)))
                error = encoder1Reading - encoder2Reading

                speed1 = speed
                speed2 = speed

                # Using master and slave solution;
                if error == 0:
                    speed1 = speed
                    speed2 = speed

                elif error < 0 and not speed == self.min_deceleration_speed:
                    speed1 += 1

                elif error > 0 and not speed == self.min_deceleration_speed:
                    speed1 -= 1

                # Drive until the encoder reaches its destination
                # Driving is split into two motors hence they are more accurate
                if encoder1Reading >= encoderDestination:
                    self.mainRobot.stop1()
                    motor1Drive = False

                else:
                    self.mainRobot.drive(speed1, 0)

                if encoder2Reading >= encoderDestination:
                    self.mainRobot.stop2()
                    motor2Drive = False

                else:
                    self.mainRobot.drive(0, speed2)

            if not motor1Drive and not motor2Drive:
                log.info("Finished driving")
                sleep(self.rest_after_drive)
                break

    def turnRobot(self, degrees, speed, direction=True, smallRobotSensors=[]):
        """
        This function turns a robot. Depending on the argument 'direction', a robot can turn right or left
        :param degrees:
        :param speed:
        :param direction:
        :param smallRobotSensors:
        :return:
        """
        self.mainRobot.reset_encoders()

        self.speed = speed

        oneWheelDistance = (self.circumferenceOfCircle / 360) * degrees

        deceleration_threshold = self.getDecelerationDistance(oneWheelDistance, speed, self.deceleration_offset)

        if direction:
            log.debug("Turn robot right: {} degrees | speed: {} | start deceleration at: {}".format(degrees, speed, deceleration_threshold))
            encoder1Destination = oneWheelDistance / self.oneEncMM
            encoder2Destination = self.encoderMaxValue - (oneWheelDistance / self.oneEncMM)

        else:
            log.debug("Turn robot left: {} degrees | speed: {} | start deceleration at: {}".format(degrees, speed, deceleration_threshold))
            encoder1Destination = self.encoderMaxValue - (oneWheelDistance / self.oneEncMM)
            encoder2Destination = oneWheelDistance / self.oneEncMM

        encoder1Reading = self.mainRobot.read_encoder1()
        encoder2Reading = self.mainRobot.read_encoder2()

        obstacleClear = True

        motor1Drive = True
        motor2Drive = True

        init_time = False

        if direction:
            while True:

                if self.checkForObstacle(smallRobotSensors, obstacleClear):

                    obstacleClear = False
                    self.mainRobot.stop1()
                    self.mainRobot.stop2()

                else:
                    obstacleClear = True

                    if encoder2Reading == 0:
                        encoder2ReadingAbs = 0

                    else:
                        encoder2ReadingAbs = self.encoderMaxValue - encoder2Reading

                    encodersAvg = (encoder2ReadingAbs + encoder1Reading) / 2.0
                    currentTravelDistance = round(encodersAvg * self.oneEncMM, 3)

                    # Control the deceleration distance and speed of deceleration
                    if currentTravelDistance >= deceleration_threshold:
                        if not init_time:
                            self.when = time.time() + self.speed_interval
                            init_time = True
                            speed = self.deceleration_control(speed)
                        else:
                            
                            speed = self.deceleration_control(speed)

                    encoder1Reading = self.mainRobot.read_encoder1()
                    encoder2Reading = self.mainRobot.read_encoder2()

                    if encoder1Reading <= encoder1Destination or encoder1Reading == 0 or encoder2Reading == 0:
                        self.mainRobot.drive(speed, 0)

                    else:
                        self.mainRobot.stop1()
                        motor1Drive = False

                    if encoder2Reading >= encoder2Destination or encoder1Reading == 0 or encoder2Reading == 0:
                        self.mainRobot.drive(0, -speed)

                    else:
                        self.mainRobot.stop2()
                        motor2Drive = False

                if not motor1Drive and not motor2Drive:
                    log.info("Finished turning right")
                    sleep(self.rest_after_drive)
                    break

        else:

            while True:

                if self.checkForObstacle(smallRobotSensors, obstacleClear):

                    obstacleClear = False
                    self.mainRobot.stop1()
                    self.mainRobot.stop2()

                else:
                    obstacleClear = True

                    if encoder1Reading == 0:
                        encoder1ReadingAbs = 0

                    else:
                        encoder1ReadingAbs = self.encoderMaxValue - encoder1Reading

                    encodersAvg = (encoder1ReadingAbs + encoder2Reading) / 2.0
                    currentTravelDistance = round(encodersAvg * self.oneEncMM, 3)

                    # Control the deceleration distance and speed of deceleration
                    if currentTravelDistance >= deceleration_threshold:
                        if not init_time:
                            self.when = time.time() + self.speed_interval
                            init_time = True

                            speed = self.deceleration_control(speed)

                        else:
                            speed = self.deceleration_control(speed)

                    encoder1Reading = self.mainRobot.read_encoder1()
                    encoder2Reading = self.mainRobot.read_encoder2()

                    if encoder1Reading >= encoder1Destination or encoder1Reading == 0 or encoder2Reading == 0:
                        self.mainRobot.drive(-speed, 0)

                    else:
                        self.mainRobot.stop1()
                        motor1Drive = False

                    if encoder2Reading <= encoder2Destination or encoder1Reading == 0 or encoder2Reading == 0:
                        self.mainRobot.drive(0, speed)

                    else:
                        self.mainRobot.stop2()
                        motor2Drive = False

                if not motor1Drive and not motor2Drive:
                    log.info("Finished turning left")
                    sleep(self.rest_after_drive)
                    break


    def driveBack(self, distance, speed, sensors=[]):
        """
        This function drives a robot ba ckward. By adjusting values such as: speed and distance can control a robot.
        :param distance:
        :param speed:
        :return:
        """
        log.info("Drive backward for: {} speed: {}".format(distance, speed))

        self.mainRobot.reset_encoders()

        distance = float(distance)
        encoderDestination = distance / self.oneEncMM

        encoder1Reading = self.mainRobot.read_encoder1()
        encoder2Reading = self.mainRobot.read_encoder2()

        obstacleClear = True

        # stoppingThresholds = self.calcStoppingDriveThreshold(speed)
        encoderDestination = self.encoderMaxValue - encoderDestination

        while encoder1Reading >= encoderDestination and encoder2Reading >= encoderDestination or encoder1Reading == 0 or encoder2Reading == 0:

            if self.checkForObstacle(sensors, obstacleClear):

                obstacleClear = False

                self.mainRobot.stop1()
                self.mainRobot.stop2()
            else:

                self.mainRobot.drive(-speed, -speed)
                obstacleClear = True

            encodersAvg = (encoder1Reading + encoder1Reading) / 2.0

            currentTravelDistance = round((self.encoderMaxValue - encodersAvg) * self.oneEncMM, 3)

            # For debug only
            # travelledDistance = self.travelledDistance(distance, currentTravelDistance)
            # speed = self.speedControlDrive(speed, travelledDistance, stoppingThresholds)

            encoder1Reading = self.mainRobot.read_encoder1()
            encoder2Reading = self.mainRobot.read_encoder2()

        else:
            self.mainRobot.stop1()
            self.mainRobot.stop2()


    def sensorTest(self, sensors, timein=10):
        """
        Test sensors, provide values such 'center', 'left', 'right' or 'all'
        :param sensorsToTest:
        :param timein:
        :return:
        """
        countdown = time.time() + timein

        endTime = time.time()

        printVals = ""

        while countdown > endTime:

            for sensor in sensors:
                printVals = printVals + "Sensor: {} value: {}  |  \n".format(sensor.position, sensor.getSensorValue())

            print(printVals)

        else:
            print("Finish sensor test!")

    def checkStatus(self):
        canRun = True

        getBatteryVoltage = self.mainRobot.battery()
        getBatteryVoltage = getBatteryVoltage / 10.0

        print("\n" + tc.FAIL + "Battery Status: " + str(getBatteryVoltage) + "V" + tc.ENDC + "\n")

        if float(getBatteryVoltage) < 10.5:
            log.error(tc.FAIL + "Critical Battery Level. PLEASE REPLACE BATTERY!" + tc.ENDC)
            canRun = False

        else:
            log.info(tc.OKGREEN + "Battery in good level." + tc.ENDC + "\n")

        return canRun

    def changeAcc(self, value=5):
        """
        This changes an acceleration if none then returs nothing and does not change the mode
        :param value:
        :return:
        """
        log.info("Changed acceleration to: {}".format(value))

        if value == None:
            return

        self.mainRobot.setAcceleration(value)

class RobotHelpers:
    def __init__(self):
        self.motorsPin = config.robotSettings['motorsPin']
        self.valvePin = config.robotSettings['valvePin']

        GPIO.setmode(GPIO.BOARD)  # choose BCM or BOARD
        GPIO.setup(self.motorsPin, GPIO.OUT)
        GPIO.setup(self.valvePin, GPIO.OUT)

        log.debug("Initialized robot helpers. (Valve and motors)")

    def motorsOn(self):
        GPIO.output(self.motorsPin, 1)
        log.debug("Start launcher motors")

    def motorsOff(self):
        GPIO.output(self.motorsPin, 0)
        log.debug("Motors launcher stopped")

    def valveRelease(self):
        k = 0

        while k < 30:
            GPIO.output(self.valvePin, 1)
            sleep(0.02)
            GPIO.output(self.valvePin, 0)
            sleep(0.06)
            k += 1

        sleep(1)

        k = 0

        while k < 3:
            GPIO.output(self.valvePin, 1)
            sleep(0.02)
            GPIO.output(self.valvePin, 0)
            sleep(0.08)
            k += 1

    def timer(self, drive):

        end_time = time.time() + 99

        while time.time() < end_time:
            sleep(0.0000001)

        drive.stopDriving()

        log.debug("Shutting down! Time is over")

        thread.interrupt_main()