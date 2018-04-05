import time
import smbus


class Sensor:
    device = smbus.SMBus(1)
    address = None
    delay_time = 0.07
    position = None

    def __init__(self, dev_addr, position):
        self.address = dev_addr
        self.position = position
        self.lastReading = None

    def getSensorValue(self):
        self.__write_i2c(0x51)
        time.sleep(self.delay_time)
        range_byte_1 = self.__read_i2c(2)
        range_byte_2 = self.__read_i2c(3)

        distance = (range_byte_1 << 8) + range_byte_2

        return distance

    def reqDistance(self):
        self.__write_i2c(0x51)

    def readDistance(self):
        range_byte_1 = self.__read_i2c(2)
        range_byte_2 = self.__read_i2c(3)

        distance = int((range_byte_1 << 8) + range_byte_2)

        self.lastReading = distance

        return distance

    def __write_i2c(self, data):
        self.device.write_byte_data(self.address, 0, data)
        return 0

    def __read_i2c(self, reg):
        data = self.device.read_byte_data(self.address, reg)
        return data