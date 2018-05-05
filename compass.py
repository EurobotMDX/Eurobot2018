#
# FishPi - An autonomous drop in the ocean
#
# Compass (Magnetometer) using CMPS10 - Tilt Compensated Compass Module
#  - Details at http://www.robot-electronics.co.uk/htm/cmps10i2c.htm
#
#  - Standard sense gives:
#   - Compass bearing between 0-359.9
#   - Pitch angle in degrees from the horizontal plane.
#   - Roll angle in degrees from the horizontal plane.
#
#  - Detailed raw sense gives:
#   - Magnetometer (X,Y,Z)
#   - Accelerometer (X,Y,Z)
#
#  - Support for Calibration, I2C address changing, factory reset, serial or PWM modes not provided.
#
#
# Adafruit i2c library (and others) at https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git

from settings import logging as log
from time import sleep
import smbus

dummy = True

try:
    import smbus
    dummy = False
    log.info('SMBUS is available')

except:
    log.info('SMBUS not available; in dummy mode')


class moving_average:

   def __init__(self, weight=0.5):
      """
      The weight (0<weight<=1) defines the proportion that the latest
      value contributes to the moving average.

      Higher values react more quickly to changes in the input data.

      Lower values smooth out outliers.
      """
      self.inited = False
      self._normalise(weight) # Make sure weight has a sensible value

   def average(self):
      """
      Retrun the current moving average.
      """
      if self.inited:
         return self.aver
      else:
         return None

   def add_value(self, value):
      """
      Add new value to moving avarage.
      """
      if self.inited:
         self.aver = ((value * self.weight) + (self.aver * (1-self.weight)))

      else:

         self.inited = True
         self.aver = value

   def reset(self, weight=0.5):
      """
      Reset moving average.
      """
      self.inited = False
      self._normalise(weight)

   def _normalise(self, weight):
      """
      Make sure weight has sensible values.
      """
      if weight <= 0:
         self.weight = 0.001
      elif weight > 1:
         self.weight = 1
      else:
         self.weight = weight


class Cmps10_Sensor:
    """ Tilt adjusted Compass sensor CMP10 over I2C. """

    def __init__(self, debug=False, address=0x60, bus=1):
        self.address = address
        self.bus = None
        self.debug = debug
        self.sensorReadings = []

        self.ma = moving_average(1)

        if not dummy:
            log.debug('Setting up SMBus')

            try:
                self.bus = smbus.SMBus(bus)

            except IOError:
                log.error("IO error. Please check if compass module is connected correctly.")

    def resetCalib(self):
        self.bus.write_byte_data(self.address, 0x22, 0xF0)
        print ("reset calibration")

    def toBinary(self, input):

        input = bin(input)[2:]

        while len(input) < 8:
            input = "0" + input

        return input

    def add(self, a, b):

        f = lambda a, b: eval("0b" + self.toBinary(a) + self.toBinary(b))

        result = f(a, b)

        return result

    # for v in range(10):
    #    ma.add_value(30+(v/10.0))
    #    print(ma.average())

    def read_sensor(self):
        """ Read sensor values. """
        total = 0
        avr = 0

        avrLength = 50

        attempts = 20

        val1 = int(self.bus.read_byte_data(self.address, 2))
        val2 = int(self.bus.read_byte_data(self.address, 3))

        total = total + self.add(val1, val2) / 10.0

        if len(self.sensorReadings) < avrLength:
            self.sensorReadings.append(total)

        else:
            self.sensorReadings = self.sensorReadings[1:-1]
            self.sensorReadings.append(total)

        for reading in self.sensorReadings:
            avr += reading

        avr = avr / len(self.sensorReadings)
        #
        # print (avr)

        print(total)

        # for i in range(0, attempts):
        #
        #     val1 = int(self.bus.read_byte_data(self.address, 2))
        #     val2 = int(self.bus.read_byte_data(self.address, 3))
        #     total = total + self.add(val1, val2) / 10.0
        #     self.ma.add_value(total)
        #     sleep(0.001)

        # print (self.ma.average())
        #
        # self.ma.reset(0.5)

        # print (self.add(val1, val2)/10.0)

        # print (type(val1))
        # print (self.int_to_bytes(val2, 3))
        # print ("\n")

        # heading = bytes(val1).encode() << bytes(val2).encode()

        # print (total/attempts)

        # print (heading)
        # heading = "%s%s" % (val1, val2)
        heading = val2

        # bearing = bus.read_byte_data(self.address, 2)

        # heading = float(self.bus.read_byte_data(self.address, 2))

        pitch = float(self.bus.read_byte_data(self.address, 4))
        roll = float(self.bus.read_byte_data(self.address, 5))

        if self.debug:
            log.debug("SENSOR:\tCMPS10\tHeading %f, pitch %f, roll %f", heading, pitch, roll)
        return heading, pitch, roll

    def read_sensor_raw(self):
        """ Read raw sensor values. """
        # read 2 registers each for raw sensor values
        # Magnetometer
        m_x = float(self.bus.read_byte_data(self.address, 10))
        m_y = float(self.bus.read_byte_data(self.address, 12))
        m_z = float(self.bus.read_byte_data(self.address, 14))

        # Accelerometer
        a_x = float(self.bus.read_byte_data(self.address, 16))
        a_y = float(self.bus.read_byte_data(self.address, 18))
        a_z = float(self.bus.read_byte_data(self.address, 20))

        if self.debug:
            log.debug("SENSOR:\tCMPS10\tRaw values: M(x,y,z)=(%f,%f,%f) A(x,y,z)=(%f,%f,%f)", m_x, m_y, m_z, a_x, a_y, a_z)

        return (m_x, m_y, m_z, a_x, a_y, a_z)


cmps = Cmps10_Sensor(debug=False)

# cmps.resetCalib()
# ma.reset(0.1) # Now 0.1 weight for current reading


# while True:

    # cmps.read_sensor()
    # print(cmps.read_sensor()[0])
    # print(c mps.read_sensor_raw())
    # sleep(0.01)