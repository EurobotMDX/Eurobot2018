import time
import smbus
import sys
sys.path.insert(0, '..')
from settings import logging as log

class Sensor:
    device = smbus.SMBus(1)
    address = None
    # delay_time = 0.4
    position = None

    def __init__(self, dev_addr, position):
        self.address = dev_addr
        self.position = position
        self.lastReading = None
        # self.delay_time = 0.07
        self.delay_time = 0.1

    def getSensorValue(self):
        self.reqDistance()
        time.sleep(self.delay_time)
        distance = self.readDistanceResponse()
        return distance


    def reqDistance(self):
        try:
            self.__write_i2c(0x51)
        except IOError:
            log.error("IO error. Sensor: {} request distance problem".format(self.position))
            return 30

    def readDistanceResponse(self):
        try:

            range_byte_1 = self.__read_i2c(2)
            range_byte_2 = self.__read_i2c(3)

            distance = int((range_byte_1 << 8) + range_byte_2)

            self.lastReading = distance

            return distance

        except IOError:
            log.error("IO error. Sensor: {} read distance problem".format(self.position))
            return 30

    def __write_i2c(self, data):
        self.device.write_byte_data(self.address, 0, data)
        return 30

    def __read_i2c(self, reg):
        data = self.device.read_byte_data(self.address, reg)
        return data