#! /usr/bin/env python

import rospy
import math
import multiprocessing
import time	
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from ctrl_pkg.msg import ServoCtrlMsg

safety = Int16()
stop = Int16()
stop.data = 0
a = []
b = []
count = Int16()
count.data = 0

def scan_callback(msg):
	pub_manual_drive = rospy.Publisher('/manual_drive', ServoCtrlMsg, queue_size = 10)
	msg1 = ServoCtrlMsg()
	temp = len(msg.ranges) #760 datas collecte
	print 'Value at 0 degree:'
	for i in range(20):
		if(((temp - 10 + i) < temp )):
			b.append(msg.ranges[temp - i - 1])
		else:
			b.append(msg.ranges[i - 10])
	f_min = min(b)
	print(f_min)
        print 'Value at 180 degree:'
	for i in range(20):
		a.append(msg.ranges[(temp / 2) -10 + i])
	b_min = min(a)
        print(b_min)

	if (f_min < 0.7):
		if safety.data != 2:
			safety.data = 2
			msg1.throttle =  2 * count.data
			msg1.angle = 0
			for i in range (600):
				pub_manual_drive.publish(msg1)
			msg1.throttle = 0.0
			msg1.angle = 0
			pub_manual_drive.publish(msg1)
	elif (b_min < 0.7):
		if safety.data != 1:
			safety.data = 1
			msg1.throttle = - 2 * count.data
			msg1.angle = 0
			for i in range (600):
				pub_manual_drive.publish(msg1)
			msg1.throttle = 0.0
			msg1.angle = 0
			pub_manual_drive.publish(msg1)
	else:
		safety.data = 0
	
	del a[:]
	del b[:]
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
	
	count.data = msg.throttle
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
