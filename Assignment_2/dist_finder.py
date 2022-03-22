#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall)
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.


    #print(len(data.ranges))
	#print(data.range_min)
	#print(data.range_max)
	#print(data.angle_increment)

	#ranges array seems have 720 slots, 720/240 = 3 slots per degree
	index = 3*(angle + 30)  # thus, 0 degrees would be at index 90
	if data.ranges[index] < data.range_min or data.ranges[index] > data.range_max:
		pass
	else:
		return data.ranges[index]



def callback(data):
	global forward_projection, error

	theta = 50 # you need to try different values for theta
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	swing = math.radians(theta)

	## Your code goes here to determine the error as per the alrorithm 
	# Compute Alpha, AB, and CD..and finally the error.
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	degree_alpha = math.degrees(alpha)

	AB = b*math.cos(alpha)
	CD = AB + (forward_projection*math.sin(alpha))

	error = CD - desired_distance

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error	
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_4/scan",LaserScan,callback)
	rospy.spin()
