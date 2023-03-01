#! /usr/bin/env python

import rospy
import math
import multiprocessing
import time	
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from ctrl_pkg.msg import ServoCtrlMsg
from simple_pid import PID

safety = Int16()
stop = Int16()
stop.data = 0
count = Int16()
count.data = 0
a = []
b = []
des_point = 0.4
current_point = 0
throttle_range = [-1 , 1]
pid = PID(1.0, 0.3, 0.1, setpoint = des_point, proportional_on_measurement = False, output_limits = throttle_range)
pid.sample_time = 0.18

def scan_callback(msg):
	pub_manual_drive = rospy.Publisher('/manual_drive', ServoCtrlMsg, queue_size = 10)
	msg1 = ServoCtrlMsg()
	temp = len(msg.ranges) #760 datas collected

	print 'Shortest distance in front of the vehicle:'
	for i in range(20):
		if(((temp - 10 + i) < temp )):
			b.append(msg.ranges[temp - i - 1])
		else:
			b.append(msg.ranges[i - 10])
	f_min = min(b)
	print(f_min)
	
    	print 'Shortest distance back of the vehicle:'
	for i in range(20):
		a.append(msg.ranges[(temp / 2) -10 + i])
	b_min = min(a)
    	print(b_min)

	if (f_min <= 0.8 ):
		current_point = f_min
		safety.data = 2
		count.data += 1
		if(count.data < 12):
			if current_point != des_point:
				throttle = pid(current_point, dt=pid.sample_time)

				msg1.throttle = throttle
				pub_manual_drive.publish(msg1)
	elif (b_min <= 0.8 ):
		current_point = b_min
		safety.data = 1
		count.data += 1
		if(count.data < 12):
			if current_point != des_point:
				throttle = pid(current_point, dt=pid.sample_time)

				msg1.throttle = -throttle
				pub_manual_drive.publish(msg1)
		
	else:
		count.data = 0
		safety.data = 0
	
	del a[:]
	del b[:]
	print("count:", count.data)
	print("safetyflag:", safety.data)


def drive_callback(data):
	pub_manual_drive = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size = 10)
	msg = ServoCtrlMsg()
	
	if 	((data.throttle > 0) and (safety.data == 2)): 		#safety program for moving forward
		msg.throttle = data.throttle
		msg.angle = data.angle
		pub_manual_drive.publish(msg)
	elif ((data.throttle < 0) and (safety.data == 1)): 	#safety program for moving backward
		msg.throttle = data.throttle
		msg.angle = data.angle
		pub_manual_drive.publish(msg)
	elif safety.data == 0:							#normal moving command
		msg.throttle = data.throttle
		msg.angle = data.angle
		pub_manual_drive.publish(msg)
	else:
		msg.throttle = 0
		msg.angle = 0
		pub_manual_drive.publish(msg)
	
	print(msg)


def listener():
	rospy.init_node('scan_values')
	sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
	sub = rospy.Subscriber('/test', ServoCtrlMsg, drive_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
