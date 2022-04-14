#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
angle = 0.0

 
# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 0.0	#TODO

# Publisher for moving the car. 
# TODO: Use the correct topic /car_x/offboard/command.
command_pub = rospy.Publisher('/car_4/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	print("PID Control Node is Listening to error")
	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	
	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering

	V_theta = (kp*data.error) + (kd*(prev_error - data.error))
	angle = angle - V_theta
	prev_error = data.error

	if angle > 100:
		angle = 100
	elif angle < -100:
		angle = -100
	
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	command.steering_angle = angle

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = vel_input

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':
	# global kp
	# global kd
	# global ki
	# global vel_input
	kp = input("Enter Kp Value: ") # good option is 14.0 to start
	kd = input("Enter Kd Value: ") # good option is 0.9 to start
	ki = input("Enter Ki Value: ") # good option is 0.0
	vel_input = input("Enter desired velocity: ") # start with 15.0
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
