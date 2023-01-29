#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
# Lib for reading data from Excel 
# from openpyxl import load_workbook
# from openpyxl import Workbook

import csv

# def getState(data):
# 	print data.data

#Define a RRBot joint positions publisher for joint controllers.
def slider_joint_positions_publisher():

	#Initiate node for controlling 4 joints positions.
	rospy.init_node('slider_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub_left_hip_pitch   = rospy.Publisher('/slider_gazebo/left_hip_pitch_position_controller/command', Float64, queue_size=None)
	pub_left_hip_roll    = rospy.Publisher('/slider_gazebo/left_hip_roll_position_controller/command', Float64, queue_size=None)
	pub_left_hip_slide   = rospy.Publisher('/slider_gazebo/left_hip_slide_position_controller/command', Float64, queue_size=None)
	pub_left_ankle_roll  = rospy.Publisher('/slider_gazebo/left_ankle_roll_position_controller/command', Float64, queue_size=None)
	pub_left_ankle_pitch = rospy.Publisher('/slider_gazebo/left_ankle_pitch_position_controller/command', Float64, queue_size=None)

	pub_right_hip_pitch   = rospy.Publisher('/slider_gazebo/right_hip_pitch_position_controller/command', Float64, queue_size=None)
	pub_right_hip_roll    = rospy.Publisher('/slider_gazebo/right_hip_roll_position_controller/command', Float64, queue_size=None)
	pub_right_hip_slide   = rospy.Publisher('/slider_gazebo/right_hip_slide_position_controller/command', Float64, queue_size=None)
	pub_right_ankle_roll  = rospy.Publisher('/slider_gazebo/right_ankle_roll_position_controller/command', Float64, queue_size=None)
	pub_right_ankle_pitch = rospy.Publisher('/slider_gazebo/right_ankle_pitch_position_controller/command', Float64, queue_size=None)

	rate = rospy.Rate(1000) #1000 Hz
	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	time_delay = 2000
	intervel = 2000
	# traj_len = len(left_hip_pitch_position)

	# subscriber
	# rospy.Subscriber("/gazebo/link_states", gazebo_msgs, getState)
	# Publish the joint trajectory
	while not rospy.is_shutdown():
		if(i <= time_delay):
			index = 0
		elif(time_delay < i and i < traj_len + time_delay):
			index = i-time_delay
		elif(i >= traj_len + time_delay):
			index = traj_len - 1

		left_hip_slide_compensation = 0.5*0.106*math.tan(-left_hip_roll_position[index])# only compensate for 50%, if 1 will be too much
		right_hip_slide_compensation = 0.5*0.106*math.tan(right_hip_roll_position[index])# only compensate for 50%, if 1 will be too much
		pub_left_hip_pitch.publish(left_hip_pitch_position[index])
		pub_left_hip_roll.publish(left_hip_roll_position[index])
		pub_left_hip_slide.publish(left_hip_slide_position[index]-left_hip_slide_compensation-0.025) # 0.025: offset from ankle pitch axis to carbon leg bottom 
		pub_left_ankle_roll.publish(left_ankle_roll_position[index])
		pub_left_ankle_pitch.publish(left_ankle_pitch_position[index])
		pub_right_hip_pitch.publish(right_hip_pitch_position[index])
		pub_right_hip_roll.publish(right_hip_roll_position[index])
		pub_right_hip_slide.publish(right_hip_slide_position[index]-right_hip_slide_compensation-0.025) # 0.025: offset from ankle pitch axis to carbon leg bottom  
		pub_right_ankle_roll.publish(right_ankle_roll_position[index])
		pub_right_ankle_pitch.publish(right_ankle_pitch_position[index])
		
		i = i+1 

		if(i > traj_len + time_delay - 5):
			i = 0
		rate.sleep() #sleep for rest of rospy.Rate(100)
		# rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':

	right_hip_pitch_position   = []
	right_hip_roll_position    = []
	right_hip_slide_position   = []
	right_ankle_roll_position  = []
	right_ankle_pitch_position = []
	left_hip_pitch_position    = []
	left_hip_roll_position     = []
	left_hip_slide_position    = []
	left_ankle_roll_position   = []
	left_ankle_pitch_position  = []

	j = 0
	filePath = '/home/harry/Documents/slider_osc/src/slider_gazebo/data/'

	with open(filePath + 'joint_trajectory_MATLAB.csv') as csvfile:
		readCSV = csv.reader(csvfile, delimiter=',')
		for row in readCSV:
			if j != 0:
				right_hip_pitch_position.append(float(row[0]))
				right_hip_roll_position.append(float(row[1]))
				right_hip_slide_position.append(float(row[2]))
				right_ankle_roll_position.append(float(row[3]))
				right_ankle_pitch_position.append(float(row[4]))
				left_hip_pitch_position.append(float(row[5]))
				left_hip_roll_position.append(float(row[6]))
				left_hip_slide_position.append(float(row[7]))
				left_ankle_roll_position.append(float(row[8]))
				left_ankle_pitch_position.append(float(row[9]))
			j = j+1

	traj_len = j-1

	try: 
		slider_joint_positions_publisher()

	except rospy.ROSInterruptException: pass