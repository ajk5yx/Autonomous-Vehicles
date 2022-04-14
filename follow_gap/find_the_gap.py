#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


command_pub = rospy.Publisher('/car_4/offboard/command', AckermannDrive, queue_size = 1)


# Good range for LIDAR (to ensure forward readings) is 100-650 (i.e. range_min = 100, range_max = 650)

def filterReadings(data):
	
	lidar_readings = [0 for q in range(551)]
	
	#this essentially removes all LIDAR measurements that go behind the car and 
	#puts the desired measurements into an array called lidar_readings
	for i in range(len(data.ranges)):
		if i >= 100 and i <= 650:
			if (math.isnan(data.ranges[i])) or (data.ranges[i] > data.range_max):
				lidar_readings[i-100] = data.range_max
			elif data.ranges[i] < data.range_min:
				lidar_readings[i-100] = data.range_min
			else:
				lidar_readings[i-100] = data.ranges[i]
	
	#this filters the data by checking for disparities, and altering the measurements 
	#in lidar_readings appropriately based on the disparities (see slides for how I did this)
	for j in range(len(lidar_readings)):
		if j+1 < len(lidar_readings):
			if abs(lidar_readings[j] - lidar_readings[j+1]) > 0.2:
				if lidar_readings[j] < lidar_readings[j+1]:
					for l in range(j - 100, j):
						if l < 0:
							l = 0
						if lidar_readings[l] > lidar_readings[j]:
							lidar_readings[l] = lidar_readings[j]
				if lidar_readings[j+1] < lidar_readings[j]:
					for l in range(j+1, j+101):
						if l > len(lidar_readings) - 1:
							l = len(lidar_readings) - 1
						if lidar_readings[l] > lidar_readings[j+1]:
							lidar_readings[l] = lidar_readings[j+1]

	return	lidar_readings

def gap_finder(data):
	
	filtered_data = filterReadings(data)
	max_value = 0.0
	max_index = 0
	middle_index = 400
	angle = 0.0
	command = AckermannDrive()

	#with the filtered data, find the farthest distance away from the LIDAR and prepare
	#to steer towards that distance (a large distance represents a lot of open space, no obstables)
	for i in range(len(filtered_data)):
		if filtered_data[i] > max_value:
			max_value = filtered_data[i]
			max_index = i

	print(max_index)
	print(middle_index)

	#this adjusts the steering angle based on the angle between the maximum distance and the center of the LIDAR
	if (max_index +100) < middle_index:
		angle = -60.0*(middle_index - (max_index+100))*data.angle_increment
		print(angle)
	if (max_index+150) > middle_index:
		angle = 60.0*((max_index+100) - middle_index)*data.angle_increment
		print(angle)

	if(angle > 100):
		angle = 100
	if(angle < -100):
		angle = -100

	#this adjusts the velocity (dynamically) based on how far away the obstacle/wall we're driving towards is
	vel_input = 7.0*max_value

	if(vel_input < 15.0):
		vel_input = 15.0

	command.steering_angle = angle
	command.speed = vel_input

	rate = rospy.Rate(150)
	# Move the car autonomously
	command_pub.publish(command)
	rate.sleep()

if __name__ == '__main__':
	rospy.init_node('find_the_gap', anonymous=True)
	rospy.Subscriber("/car_4/scan",LaserScan,gap_finder)
	rospy.spin()
