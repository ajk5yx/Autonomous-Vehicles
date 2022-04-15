#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


command_pub = rospy.Publisher('/car_4/offboard/command', AckermannDrive, queue_size = 1)


# Good range for LIDAR (to ensure forward readings) is 175-600 (i.e. range_min = 175, range_max = 600)

def filterReadings(data):
	
	lidar_readings = [0 for q in range(426)]
	discrepencies = []							#will contain a tuple of tuples with the interiror tuples being each measurement in a discrepency
												#and it's corresponding index in the array

	#this essentially removes all LIDAR measurements that go behind the car and 
	#puts the desired measurements into an array called lidar_readings
	for i in range(len(data.ranges)):
		if i >= 175 and i <= 600:
			if (math.isinf(data.ranges[i])) or (data.ranges[i] > data.range_max):
				lidar_readings[i-175] = data.range_max
			elif math.isnan(data.ranges[i]) or (data.ranges[i] < data.range_min):
				lidar_readings[i-175] = data.range_min
			else:
				lidar_readings[i-175] = data.ranges[i]
	
	#this filters the data by checking for disparities, and altering the measurements 
	#in lidar_readings appropriately based on the disparities (see slides for how I did this)
	for j in range(len(lidar_readings)):
		if j+1 < len(lidar_readings):
			# the is not range_min is mainly accounting for nan measurements from the LIDAR, trying to reduce random LIDAR errors
			if abs(lidar_readings[j] - lidar_readings[j+1]) > 0.2 and (lidar_readings[j] != data.range_min) and (lidar_readings[j+1] != data.range_min):
				discrepencies.append(((j, lidar_readings[j]), (j+1, lidar_readings[j+1])))
	
	#print statement used for debugging
	# print(discrepencies)	

	#this implements the disparity extender algorithm, stretching the closer measurement
	# of the two measurements in the discrepency by approximately half the width
	# of the car in the direction of the larger measurement(i.e. creating a safety bubble)			
	closest_discrepency = ()
	for m in range(len(discrepencies)):
		if m == 0:
			closest_discrepency = discrepencies[0]
		elif (discrepencies[m][0][1] or discrepencies[m][1][1]) < (closest_discrepency[0][1] or closest_discrepency[1][1]):
			closest_discrepency = discrepencies[m]
		

	distance_to_handle_bubbles = min(closest_discrepency[0][1], closest_discrepency[1][1]) + 0.75
	ones_we_care_about = []
	for n in range(len(discrepencies)):
		if (discrepencies[n][0][1] or discrepencies[n][1][1]) < distance_to_handle_bubbles:
			ones_we_care_about.append(discrepencies[n])

	# print(ones_we_care_about)

	#THIS IS THE PART THAT NEEDS TO BE TWEAKED; EVERYTHING ELSE SHOULD BE FINE
	for o in range(len(ones_we_care_about)):
		if ones_we_care_about[o][0][1] < ones_we_care_about[o][1][1]:
			# angle = math.atan(0.2/ones_we_care_about[o][0][1])
			# indices_to_move_over = int(angle/data.angle_increment) + 20

			#THIS IF STATEMENT IS PROBABLY WHAT NEEDS TWEAKING
			if ones_we_care_about[0][0] > 175:	# 350 (center of LIDAR ranges array - half width of car) - 175 (amount that LIDAR readings is offset by)
				indices_to_move_over = (ones_we_care_about[0][0] - 175) + 20
			else:
				indices_to_move_over = 20 #saftey bubble just in case (in this state, the car should already be clear of the obstacle)
			
			for l in range(ones_we_care_about[o][0][0]-indices_to_move_over, ones_we_care_about[o][0][0]+indices_to_move_over+1):
				print(l)
				if l < 0:
					l = 0
				if l > len(lidar_readings) - 1:
					l = len(lidar_readings) - 1
				lidar_readings[l] = 0.0
		elif ones_we_care_about[o][1][1] < ones_we_care_about[o][0][1]:
			# angle = math.atan(0.2/ones_we_care_about[o][1][1])
			# indices_to_move_over = int(angle/data.angle_increment) + 20

			# indices_to_move_over = 10*(data.range_max-ones_we_care_about[o][0][1]) + 50

			#THIS IF STATEMENT IS PROBABLY WHAT NEEDS TWEAKING
			if ones_we_care_about[1][0] < 275:		# 450 (center of lidar ranges + half width of car) - 175 (offset of lidar_readings)
				indices_to_move_over = (275 - ones_we_care_about[1][0]) + 20
			else:
				indices_to_move_over = 20 #saftey bubble just in case (in this state, the car should already be clear of the obstacle)

			for l in range(ones_we_care_about[o][0][0]-indices_to_move_over, ones_we_care_about[o][0][0]+indices_to_move_over+1):
				print(l)
				if l < 0:
					l = 0
				if l > len(lidar_readings) - 1:
					l = len(lidar_readings) - 1
				lidar_readings[l] = 0.0

	return	lidar_readings

def gap_finder(data):
	
	filtered_data = filterReadings(data)
	max_value = 0.0				#how far away the farthest measurement in filtered_data is (used for dynamic velocity)
	max_index = 0				#index in filtered_data at which the farthest away measurement is taken
	middle_index = 400			#roughly the index at which the LIDAR points dead ahead
	angle = 0.0
	command = AckermannDrive()

	#with the filtered data, find the farthest distance away from the LIDAR and prepare
	#to steer towards that distance (a large distance represents a lot of open space, no obstables)
	for i in range(len(filtered_data)):
		if filtered_data[i] > max_value:
			max_value = filtered_data[i]
			max_index = i

	#print statements used for debugging
	print(max_index + 175)
	print(max_value)
	print(middle_index)

	#this adjusts the steering angle based on the angle between the maximum distance and the center of the LIDAR
	#the +175 to max_index is required, because middle_index is taken directly from the LIDAR ranges array whereas
	#max_index is taken from filtered_data (which starts when the LIDAR index is >=175)
	if (max_index +175) < middle_index:
		angle = -80.0*(middle_index - (max_index+175))*data.angle_increment
		print(angle)
	if (max_index+175) > middle_index:
		angle = 80.0*((max_index+175) - middle_index)*data.angle_increment
		print(angle)

	if(angle > 100):
		angle = 100
	if(angle < -100):
		angle = -100

	#this adjusts the velocity (dynamically) based on how far away the obstacle/wall we're driving towards is
	vel_input = 6.0*max_value
	
	# if(vel_input < 15.0):
	# 	vel_input = 15.0

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