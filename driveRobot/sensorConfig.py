import smbus


class SensorConfig:
    device = smbus.SMBus(1)
    address = None

    def __init__(self, dev_addr):
        self.address = dev_addr

    def changeAddress(self, newAddress):
        self.device.write_byte_data(self.address, 0, 0xA0)
        self.device.write_byte_data(self.address, 0, 0xAA)
        self.device.write_byte_data(self.address, 0, 0xA5)
        self.device.write_byte_data(self.address, 0, newAddress)


    def changeRangeTime(self, newValue):
        self.device.write_byte_data(self.address, 2, newValue)

    def changeGain(self, newValue):
        self.device.write_byte_data(self.address, 1, newValue)
