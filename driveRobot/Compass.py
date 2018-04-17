import time
import math
import smbus
import sys
sys.path.insert(0, '..')
from settings import logging as log

class Compass:
    bus = smbus.SMBus(1)
    addr = None
    yoffset = - 350
    xoffset = -144

    def __init__(self, dev_addr):
        self.address = dev_addr
        self.setMode(0x00) # Set to Continuous reading mode

    def heading(self):
        xmsb = self.bus.read_byte_data(self.addr, 0x03)
        xlsb = self.bus.read_byte_data(self.addr, 0x04)
        ymsb = self.bus.read_byte_data(self.addr, 0x07)
        ylsb = self.bus.read_byte_data(self.addr, 0x08)
        x = xlsb + (xmsb << 8)
        y = ylsb + (ymsb << 8)
        # time.sleep(.250) Commented by Timbo to see if it works without

        # might need to switch to
        # if result > 32767: result -= 65536
        if x > 32767: x = -((65535 - x) + 1)
        if y > 32767: y = -((65535 - y) + 1)
        x = x - self.xoffset
        y = y - self.yoffset
        heading = math.atan2(y, x)
        if heading < 0:
            heading += 2 * math.pi
        heading = math.degrees(heading)
        return heading

    def setMode(self, mode):
        self.device.write_byte_data(self.address, 0x02, mode)

    def test(self):

        while True:
            print ("Heading {}".format(heading()))



if __name__ == '__main__':
    c = Compass()