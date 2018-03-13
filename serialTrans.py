#!/usr/bin/env python

import time
import serial

ser = serial.Serial(

    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

counter = 0

while 1:
    try:

        # ser.write("Write counter: %d \n" % (counter))

        # print("Sent data: %d\n" % (counter))

        received = ser.readline()

        if received == "":
            print ("Empty")
            break

        time.sleep(0.01)

        # counter += 1


        # received1 = received.decode("utf-8")
        # received1 =received.decode('utf-8')
        # received = received.decode('unicode_escape')

        print("Receive data: {} \n".format(received))
        # print("Receive data: {} \n".format(received.decode('utf-8')))

    except Exception as error:
        print(error)