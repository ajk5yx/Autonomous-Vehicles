#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


command_pub = rospy.Publisher('/car_4/offboard/command', AckermannDrive, queue_size = 1)
vel_input = 0.0

# Good range for LIDAR (to ensure forward readings) is 175-600 (i.e. range_min = 175, range_max = 600)
# Center of LIDAR array (pointing directly fowards) is approximately at index 400

#NOTE: I used the word "discrepency" when I mean "disparity" but I used "discrepency" 
# so many times at this point it's probably not worth changing. 
# But by discrepency I mean disparity.

def filterReadings(data):
	global vel_input
	lidar_readings = [0 for q in range(426)]

	discrepencies = []		#will contain a several tuples of tuples with the interiror tuples
							#being each measurement in a disparity and it's corresponding 
							#index in the array. Will be formatted (index, distance)

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

	
	#this filters the data by checking for disparities (distance between points > 0.1 meters),
	#and puts the disparities into the "discrepencies" list (see comment where list is initialized for format)
	for j in range(len(lidar_readings)):
		if j+1 < len(lidar_readings):
			# the "!= range_min" is accounting for nan measurements from the LIDAR, removing random LIDAR errors
			if abs(lidar_readings[j] - lidar_readings[j+1]) > 0.1 and (lidar_readings[j] != data.range_min) and (lidar_readings[j+1] != data.range_min):
				discrepencies.append(((j, lidar_readings[j]), (j+1, lidar_readings[j+1])))
	

	#print statement used for debugging
	# print(discrepencies)	

	
	#this checks through all the disparities and finds that corresponds to the closest 
	#obstacle to the car. 
	# 
	#if there are no disparities (in a straight line for example),
	#we created an invisible obstacle to force the car to turn ever so slightly (this mainly
	#addresses the problem where the car heads straight for the corner, thus causing it to crash)
	closest_discrepency = ()
	for m in range(len(discrepencies)):
		if m == 0:
			closest_discrepency = discrepencies[0]
		elif (discrepencies[m][0][1] or discrepencies[m][1][1]) < (closest_discrepency[0][1] or closest_discrepency[1][1]):
			closest_discrepency = discrepencies[m]
	
	if (len(closest_discrepency) == 0):
		closest_discrepency = ((285, 2.1), (286, 2.7))

	#print statement used for debugging
	#print(closest_discrepency)

	#This adjusts the velocity based on the distance the car is from the closest obstacle.
	#The closer the car is to an obstacle, the slower it goes to give it the best chance to
	#avoid the obstacle. Also, the minimum speed is set to 12 (because the car requires a 
	#certain amount of speed to start), and down the straights, it tops out at a speed 
	#of 30 for safety
	vel_input = 10 * min(closest_discrepency[0][1], closest_discrepency[1][1])
	if vel_input < 12:
		vel_input = 12
	if(min(closest_discrepency[0][1], closest_discrepency[1][1]) > 2.0):
		vel_input = 30
	if vel_input > 30:
		vel_input = 30

	#This filters the disparities into a list of those we actually care about. It makes sure
	#we only handle the disparities within a certain distance (0.75 meters) to fix the problem
	#where if two boxes are put behind one another such that the car should weave between them,
	#it will only pick up the second box once it is past the first (i.e. allowing it go between them)
	if(min(closest_discrepency[0][1], closest_discrepency[1][1]) + 0.75 < data.range_max):
		distance_to_handle_bubbles = min(closest_discrepency[0][1], closest_discrepency[1][1]) + 0.75
	else:
		distance_to_handle_bubbles = data.range_max

	ones_we_care_about = []
	for n in range(len(discrepencies)):
		if (discrepencies[n][0][1] < distance_to_handle_bubbles) or (discrepencies[n][1][1] < distance_to_handle_bubbles):
			ones_we_care_about.append(discrepencies[n])

	#print statement used for debugging
	#print(ones_we_care_about)

	#This is where a lot of the heavy lifting is done. This goes through the disparities
	#that matter to us and creates appropriate safety bubbles based on the position of the car
	#relative to the pair of points representing the disparity. Is far away from the obstacle
	#there is a smaller safety bubble (but never smaller than half the width of the car, unless
	#the car is already to the left or right of the obstacle), and if the car is close to the 
	#obstacle the safety bubble is bigger (forcing the car to choose a path before it potentially
	#runs into the obstacle)
	#
	#There are a lot of elements dealt with, but long story short, it creates the saftey bubbles
	for o in range(len(ones_we_care_about)):
		if ones_we_care_about[o][0][1] < ones_we_care_about[o][1][1]:
			if ones_we_care_about[o][0][0] > 175:	# 350 (center of LIDAR ranges array - half width of car) - 175 (amount that LIDAR readings is offset by)
				if ones_we_care_about[o][0][0] >= 225:
					indices_to_move_over = int(0.5*(ones_we_care_about[o][0][0] - 225)) + 50	
				else:
					indices_to_move_over = (ones_we_care_about[o][0][0]-175) + 40	
			else:
				indices_to_move_over = 40 

			for l in range(ones_we_care_about[o][0][0], ones_we_care_about[o][0][0]+indices_to_move_over+1):
				if l < 0:
					l = 0
				if l > len(lidar_readings) - 1:
					l = len(lidar_readings) - 1
				lidar_readings[l] = 0.0

		elif ones_we_care_about[o][1][1] < ones_we_care_about[o][0][1]:
			if ones_we_care_about[o][1][0] < 275:		# 450 (center of lidar ranges + half width of car) - 175 (offset of lidar_readings)
				if ones_we_care_about[o][1][0] <= 225:
					indices_to_move_over = int(0.5*(225 - ones_we_care_about[o][1][0])) + 50
				else:
					indices_to_move_over = (275 - ones_we_care_about[o][1][0]) + 40
			else:
				indices_to_move_over = 40

			for l in range(ones_we_care_about[o][0][0]-indices_to_move_over, ones_we_care_about[o][0][0]+1):
				if l < 0:
					l = 0
				if l > len(lidar_readings) - 1:
					l = len(lidar_readings) - 1
				lidar_readings[l] = 0.0

	return lidar_readings

def gap_finder(data):
	global vel_input
	filtered_data = filterReadings(data) #filtered LIDAR measurements once disparities are properlly dealt with
	max_value = 0.0	#how far away the farthest measurement in filtered_data is (used for dynamic velocity)
	max_index = 0	#index in filtered_data at which the farthest away measurement is taken
	middle_index = 400	#roughly the index at which the LIDAR points straight ahead
	angle = 0.0
	command = AckermannDrive()

	#with the filtered data, find the farthest distance away from the LIDAR and prepare
	#to steer towards that distance
	for i in range(len(filtered_data)):
		if filtered_data[i] > max_value:
			max_value = filtered_data[i]
			max_index = i

	#print statements used for debugging
	# print(max_index + 175)
	# print(max_value)
	# print(middle_index)

	#this adjusts the steering angle based on the angle between the maximum distance and the center of the LIDAR
	#the "+ 175" to max_index is required, because middle_index is taken directly from the LIDAR ranges array whereas
	#max_index is taken from filtered_data (which starts when the LIDAR index is >=175)
	if (max_index +175) < middle_index:
		angle = -80.0*(middle_index - (max_index+175))*data.angle_increment
		
	if (max_index+175) > middle_index:
		angle = 80.0*((max_index+175) - middle_index)*data.angle_increment
		

	if(angle > 100):
		angle = 100
	if(angle < -100):
		angle = -100

	#updates the steering angle and speed of the car
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