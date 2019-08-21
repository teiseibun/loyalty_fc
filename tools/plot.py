import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import serial
import struct
from collections import deque

ser = serial.Serial(
    port='/dev/ttyUSB0',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
line = []

while True:
    while ser.inWaiting() > 0:
        buffer = []
        checksum = 0

        c = ser.read(1)

        #wait for start byte
        if c == '@':
            print('start byte received')
        else:
            continue

        #receive pakage size
        payload_count, =  struct.unpack("B", ser.read(1))
        print('payload size: %d' %(payload_count))

        #receive payload and calculate checksum
        for i in range(0, payload_count):
            while ser.inWaiting() == 0:
                continue
            buffer.append(ser.read(1))
            buffer_checksum ,= struct.unpack("B", buffer[i])
            checksum ^= buffer_checksum

        #Receive the checksum byte then exam it
        checksum_byte ,= struct.unpack("B", ser.read())
        if checksum_byte != checksum:
                print("error: checksum mismatch");
                #return 'fail'
        else:
                print("checksum is correct (%d)" %(checksum))

        continue

        for i in range(0, self.line_count):
            #unpack received datas
            float_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
            #self.analog_data[i].add(np.asarray(struct.unpack("f", float_data)))
