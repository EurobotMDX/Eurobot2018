import smbus
import time
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x12

def writeNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1



# import RPi.GPIO as GPIO
# import smbus
#

# i2c = smbus.SMBus(1)

slaveAddress = 0x12
numberInterruptPIN = 19
messageInterruptPIN = 12

smsMessage = ""
smsNumber = ""


def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number


def readNumberFromArduino():
    global smsNumber
    data_received_from_Arduino = bus.read_i2c_block_data(slaveAddress, 0,11)
    for i in range(len(data_received_from_Arduino)):
        smsNumber += chr(data_received_from_Arduino[i])

    print(smsNumber)
    # print(smsNumber.encode("utf-8"))
    data_received_from_Arduino = ""
    smsNumber = ""



while True:

    try:
        var = 100

        writeNumber(var)

        print("Sent var {}".format(var))
        # print("RPI: Hi Arduino, I sent you ", var)
        # sleep one second
        time.sleep(0.001)

        number = readNumberFromArduino()

    except IOError:
            print("IO error")


# else:
#     print("Hehe")