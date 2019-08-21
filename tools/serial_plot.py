import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import serial
import struct
from collections import deque

ser = serial.Serial(
    port='/dev/ttyUSB1',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
line = []

class AnalogData:
    def __init__(self, max_count):
	self.max_count = max_count
	self.data = deque([0.0] * max_count)

    def add(self, value):
	if len(self.data) < self.max_count:
		self.data.append(val)
	else:
		self.data.pop()
	self.data.appendleft(value)

class AnalogPlot:
    def create_line(self, label_name, line_color):
	for i in range(0, len(self.line_numbers)):
		if self.line_numbers[i] == self.current_line_count:
			self.line.append(plt.plot(self.analog_data[self.current_line_count].data, \
				label=label_name, color=line_color, animated=True)[0])

	self.current_line_count += 1

    def show_subplot(self):
	plt.grid()
	plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), \
		ncol=3, fancybox=True, shadow=True)

    def animate(self, i):
	for index in range(0, len(self.line_numbers)):
		self.line[index].set_ydata( \
			self.analog_data[self.line_numbers[index]].data)

	return self.line
			
    def __init__(self, line_count, analog_data):
	self.figure = plt.figure(figsize=(14,8))
	self.line = []
	self.line_count = line_count
	self.current_line_count = 0
	self.analog_data = analog_data
	self.line_numbers = []

    def set_figure(self):		
	plt.subplot(511)
	plt.ylabel('Acceleration (g)')
	plt.ylim([-1.0, 2.0])
	self.create_line('x axis (raw data)', 'red')		
	self.create_line('y axis (raw data)', 'blue')		
	self.create_line('z axis (raw data)', 'green')		
	self.create_line('x axis (filtered data)', 'orange')		
	self.create_line('y axis (filtered data)', 'yellow')	
	
	self.create_line('z axis (filtered data)', 'purple')		
	self.show_subplot()

	plt.subplot(512)
	plt.ylabel('Degree per second (dps)')
	plt.ylim([-450, 450])
	self.create_line('x axis (raw data)', 'red')		
	self.create_line('y axis (raw data)', 'blue')		
	self.create_line('z axis (raw data)', 'green')		
	self.create_line('x axis (filtered data)', 'orange')		
	self.create_line('y axis (filtered data)', 'yellow')		
	self.create_line('z axis (filtered data)', 'purple')		
	self.show_subplot()

	plt.subplot(513)
	plt.ylabel('Attitude (degree)')
	plt.ylim([-90, 360])
	self.create_line('Roll (accelerometer)', 'red')		
	self.create_line('Pitch (accelerometer)', 'blue')		
	self.create_line('Yaw (magnetometer)', 'green')		
	self.show_subplot()

	plt.subplot(514)
	plt.ylabel('Attitude (degree)')
	plt.ylim([-90, 90])
	self.create_line('Roll (gyroscope)', 'red')		
	self.create_line('Pitch (gyroscope)', 'blue')		
	self.create_line('Yaw (gyroscope)', 'green')		
	self.show_subplot()

	plt.subplot(515)
	plt.ylabel('Complementry filter')
	plt.ylim([0.95, 1.05])
	self.create_line('alpha (roll)', 'red')		
	self.create_line('alpha (pitch)', 'blue')		
	self.show_subplot()

    def set_show_line(self, line_numbers):
	self.line_numbers = line_numbers
	self.set_figure()

    def show(self):
	ani = animation.FuncAnimation(self.figure, self.animate, np.arange(0, 200), \
		interval=0, blit=True)

	plt.show()


    def read_new_data(self):
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

            #continue

            for i in range(0, payload_count / 4):
                #unpack received data
                float_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
                unpacked_data = np.asarray(struct.unpack("f", float_data))
                print("received: %f" %(unpacked_data))
                self.analog_data[i].add(np.asarray(struct.unpack("f", float_data)))
            break

        return 'success'

#Analog plot
analog_data = [AnalogData(200) for i in range(0, 20)]
analog_plot = AnalogPlot(20, analog_data)

class SerialReadThread(threading.Thread):
	def run(self):
		while True:
			analog_plot.read_new_data()

#SerialReadThread().start()
while True:
    analog_plot.read_new_data()

#analog_plot.set_show_line([i for i in range(0, 20)])
#analog_plot.show()
