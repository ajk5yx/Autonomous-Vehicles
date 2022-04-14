#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

vel_input = 0.0
command_pub = rospy.Publisher('/car_4/offboard/command', AckermannDrive, queue_size = 1)


# Good range for LIDAR (to ensure forward readings) is 150-600 (i.e. range_min = 150, range_max = 600)

def filterReadings(data):
	
	lidar_readings = [0 for q in range(451)]
	
	for i in range(len(data.ranges)):
		if i >= 150 and i <= 600:
			if (math.isnan(data.ranges[i])) or (data.ranges[i] > data.range_max):
				lidar_readings[i-150] = data.range_max
			elif data.ranges[i] < data.range_min:
				lidar_readings[i-150] = data.range_min
			else:
				lidar_readings[i-150] = data.ranges[i]
	
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

	for i in range(len(filtered_data)):
		if filtered_data[i] > max_value:
			max_value = filtered_data[i]
			max_index = i

	print(max_index)
	print(middle_index)
	if (max_index +150) < middle_index:
		angle = -60.0*(middle_index - (max_index+150))*data.angle_increment
		print(angle)
	if (max_index+150) > middle_index:
		angle = 60.0*((max_index+150) - middle_index)*data.angle_increment
		print(angle)

	if(angle > 100):
		angle = 100
	if(angle < -100):
		angle = -100

	command.steering_angle = angle
	command.speed = vel_input

	rate = rospy.Rate(150)
	# Move the car autonomously
	command_pub.publish(command)
	rate.sleep()

if __name__ == '__main__':
	vel_input = input("Enter desired velocity: ")  # good straight line speed = 40
	rospy.init_node('find_the_gap', anonymous=True)
	rospy.Subscriber("/car_4/scan",LaserScan,gap_finder)
	rospy.spin()
