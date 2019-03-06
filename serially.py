#!/usr/bin/python

import serial
import syslog
import time

#The following line is for serial over GPIO
serial_port = '/dev/cu.usbmodem14201'
baud_rate = 9600
ser = serial.Serial(serial_port, baud_rate, timeout=.1)
#
# with open("data.txt", 'w+') as f:
#     while True:
#         line = ser.readline()
#         f.writelines(line.strip())
# exit()
