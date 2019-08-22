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

class serial_data_class:
    def __init__(self, max_count):
	self.max_count = max_count
	self.data = deque([0.0] * max_count)

    def add(self, value):
	if len(self.data) < self.max_count:
		self.data.append(val)
	else:
		self.data.pop()
	self.data.appendleft(value)

class serial_plotter_class:
    def __init__(self, curve_count, serial_data):
	self.figure = plt.figure(figsize=(14,8))
	self.curve = []
	self.curve_count = curve_count
	self.current_curve_count = 0
	self.serial_data = serial_data
	self.curve_numbers = []
    
    def create_curve(self, label_name, curve_color):
	for i in range(0, len(self.curve_numbers)):
		if self.curve_numbers[i] == self.current_curve_count:
			self.curve.append(plt.plot(self.serial_data[self.current_curve_count].data, \
				label=label_name, color=curve_color, animated=True)[0])

	self.current_curve_count += 1

    def show_subplot(self):
	plt.grid()
	plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), \
		ncol=3, fancybox=True, shadow=True)

    def animate(self, i):
	for index in range(0, len(self.curve_numbers)):
		self.curve[index].set_ydata( \
			self.serial_data[self.curve_numbers[index]].data)

	return self.curve

    def set_figure(self):		
	plt.subplot(411)
	plt.ylabel('accel [m/s^2]')
	plt.ylim([-1.0, 2.0])
	self.create_curve('x (raw)', 'red')		
	self.create_curve('y (raw)', 'blue')		
	self.create_curve('z (raw)', 'green')		
	self.show_subplot()

        plt.subplot(412)
	plt.ylabel('accel [m/s^2]')
	plt.ylim([-1.0, 2.0])
	self.create_curve('x (moving average)', 'red')		
	self.create_curve('y (moving average)', 'blue')		
	self.create_curve('z (moving average)', 'green')		
	self.show_subplot()

	plt.subplot(413)
	plt.ylabel('gyro [degree/s]')
	plt.ylim([-450, 450])
	self.create_curve('x (raw)', 'red')		
	self.create_curve('y (raw)', 'blue')		
	self.create_curve('z (raw)', 'green')		
	self.show_subplot()

        plt.subplot(414)
	plt.ylabel('gyro [degree/s]')
	plt.ylim([-450, 450])
	self.create_curve('x (moving average)', 'red')		
	self.create_curve('y (moving average)', 'blue')		
	self.create_curve('z (moving average)', 'green')		
	self.show_subplot()

    def set_curve(self, curve_numbers):
	self.curve_numbers = curve_numbers
	self.set_figure()

    def show_graph(self):
	ani = animation.FuncAnimation(self.figure, self.animate, np.arange(0, 200), \
		interval=0, blit=True)

	plt.show()

    def serial_receive(self):
        while ser.inWaiting() > 0:
            buffer = []
            checksum = 0

            c = ser.read(1)

            #wait for start byte
            if c == '@':
                print('start byte received')
            else:
                continue

            #receive package size
            while ser.inWaiting() == 0:
                continue
            payload_count, =  struct.unpack("B", ser.read(1))
            print('payload size: %d' %(payload_count))

            #receive payload and calculate checksum
            for i in range(0, payload_count):
                while ser.inWaiting() == 0:
                    continue
                buffer.append(ser.read(1))
                buffer_checksum ,= struct.unpack("B", buffer[i])
                checksum ^= buffer_checksum

            #checksum test
            checksum_byte ,= struct.unpack("B", ser.read())
            if checksum_byte != checksum:
                    print("error: checksum mismatch");
                    return 'fail'
            else:
                    print("checksum is correct (%d)" %(checksum))

            for i in range(0, payload_count / 4):
                #unpack received data
                binary_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
                float_data = np.asarray(struct.unpack("f", binary_data))
                self.serial_data[i].add(float_data)
                print("received: %f" %(float_data))
            print("-----------------------------");
            return 'success'

#Analog plot
serial_data = [serial_data_class(200) for i in range(0, 12)]
serial_plotter = serial_plotter_class(12, serial_data)

class serial_thread(threading.Thread):
	def run(self):
		while True:
			serial_plotter.serial_receive()

print("connected to: " + ser.portstr)
serial_thread().start()
serial_plotter.set_curve([i for i in range(0, 12)])
serial_plotter.show_graph()
