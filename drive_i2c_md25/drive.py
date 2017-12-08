import time

dummy = True
try:
    import smbus

    dummy = False
    print
    'smbus is available'
except:
    print
    'smbus not available; in dummy mode'

import time

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
# MD25_REGISTER_RESET_ENCODERS = 0x10



class md25:
    def __init__(self, mode=MD25_DEFAULT_MODE, bus=1, address=MD25_DEFAULT_ADDRESS):
        self.mode = mode
        self.address = address
        self.bus = None
        print
        'dummy is', dummy
        if not dummy:
            print
            'setting up SMBus'
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
                time.sleep(5)
            if turn:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, turn)
                time.sleep(5)

    def stop(self):
        print('STOP!!!')
        if (0 == self.mode or 2 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 128)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 128)
        if (1 == self.mode or 3 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 0)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 0)

    def battery(self):
        if self.bus:
            return self.bus.read_byte_data(self.address, MD25_REGISTER_BATTERY_VOLTS)
        else:
            return 120

    def read_encoder1(self):
        if self.bus:
            e1= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1A)
            e2= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1B)
            e3= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1C)
            e4= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC1D)
            return [e1, e2, e3, e4]
        else:
            return "Error while reading encder for motor 1"

    def read_encoder2(self):
        if self.bus:
            e1= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2A)
            e2= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2B)
            e3= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2C)
            e4= self.bus.read_byte_data(self.address, MD25_REGISTER_ENC2D)
            return [e1, e2, e3, e4]
        else:
            return "Error while reading encder for motor 1"

    def reset_encoders(self):
        if self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_COMMAND, 0x20)
            print ("Encoders were reset")
        else:
            print ("Could not reset encoders")


import RPi.GPIO as GPIO

def measure():
    GPIO.setmode(GPIO.BOARD)

    TRIG = 23
    ECHO = 24

    print ("starting measurment..")

    GPIO.setup(TRIG, GPIO.OUT)

    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, False)
    time.sleep(2)
    GPIO.output(TRIG,True)

    time.sleep(0.00001)

    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
            pass

    start = time.time()

    while GPIO.input(ECHO) == 1:
            pass

    stop = time.time()

    distance = ((stop - start) * 17000)

    GPIO.cleanup()

    return distance

start = md25(mode=1)


e1=time.time() + 1
e2=time.time() + 3
e3=time.time() + 4

e4=time.time() + 25


start.reset_encoders()
#
# while ((start.read_encoder1() <= [0, 0, 1, 100]) and (start.read_encoder2() <= [0, 0, 1, 100])):
#     start.drive(2, 2)
#     print(start.read_encoder1())
#     print(start.read_encoder2())
#
# else:
#     start.stop()
#     print("\n After stop print encoders")
#     print(start.read_encoder1())
#     print(start.read_encoder2())



# while(time.time() < e4):
#     print("\n After stop print encoders")
#     print(start.read_encoder1())
#     print(start.read_encoder2())
#     time.sleep(1)

while (time.time() < e4):
    print (measure())
    time.sleep(1)