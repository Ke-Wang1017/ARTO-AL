#!/usr/bin/env python

import rospy
import math
import time


from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
from math import sin,cos,atan2,sqrt,fabs,pi
from tf.transformations import euler_from_quaternion


#Define a Slider joint positions publisher for joint controllers.
def slider_joint_torques_publisher():


	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('slider_joint_torque_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub_left_hip_pitch = rospy.Publisher('/time_slider_gazebo/left_hip_pitch_torque_controller/command', Float64, queue_size=None)
	pub_left_hip_slide = rospy.Publisher('/time_slider_gazebo/left_hip_slide_torque_controller/command', Float64, queue_size=None)
	pub_left_hip_roll = rospy.Publisher('/time_slider_gazebo/left_hip_roll_torque_controller/command', Float64, queue_size=None)
	pub_left_ankle_pitch = rospy.Publisher('/time_slider_gazebo/left_ankle_pitch_torque_controller/command', Float64, queue_size=None)
	pub_left_ankle_roll = rospy.Publisher('/time_slider_gazebo/left_ankle_roll_torque_controller/command', Float64, queue_size=None)

	pub_right_hip_pitch = rospy.Publisher('/time_slider_gazebo/right_hip_pitch_torque_controller/command', Float64, queue_size=None)
	pub_right_hip_slide = rospy.Publisher('/time_slider_gazebo/right_hip_slide_torque_controller/command', Float64, queue_size=None)
	pub_right_hip_roll = rospy.Publisher('/time_slider_gazebo/right_hip_roll_torque_controller/command', Float64, queue_size=None)
	pub_right_ankle_pitch = rospy.Publisher('/time_slider_gazebo/right_ankle_pitch_torque_controller/command', Float64, queue_size=None)
	pub_right_ankle_roll = rospy.Publisher('/time_slider_gazebo/right_ankle_roll_torque_controller/command', Float64, queue_size=None)


	rate = rospy.Rate(1000) #1000 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():
		traj = [-5.45862,
              -4.99658e-18,
              51.4964,
              -4.99658e-18,
              -7.77808e-19,
              5.45862,
              -4.99658e-18,
              51.4964,
              -4.99658e-18,
              -7.77808e-19]

        pub_left_hip_roll.publish(traj[0])
        pub_left_hip_pitch.publish(traj[1])
        pub_left_hip_slide.publish(traj[2])
        pub_left_ankle_pitch.publish(traj[3])
        pub_left_ankle_roll.publish(traj[4])

        pub_right_hip_roll.publish(traj[5])
        pub_right_hip_pitch.publish(traj[6])
        pub_right_hip_slide.publish(traj[7])
        pub_right_ankle_pitch.publish(traj[8])
        pub_right_ankle_roll.publish(traj[9])

        rate.sleep() #sleep for rest of rospy.Rate(100)


		#Publish the same sine movement to each joint.
		# if i < 10000:
		# 	pub1.publish(2.5*torque*1000/(i+1))
		# 	pub3.publish(1.2*torque*1000/(i+1))
		# 	pub2.publish(4.1*torque/9.81*1000/(i+1))
		# 	pub4.publish(4.1*torque/9.81*1000/(i+1))
		# else:
		# 	gazebo_link_states()

		
		# pub_left_hip_pitch.publish(sine_movement)
		# pub_left_hip_slide.publish(sine_movement)
		# pub_left_hip_roll.publish(sine_movement)
		# pub_left_ankle_pitch.publish(sine_movement)

		# pub_left_ankle_roll.publish(sine_movement)
		# pub_right_hip_pitch.publish(sine_movement)
		# pub_right_hip_slide.publish(sine_movement)
		# pub_right_hip_roll.publish(sine_movement)
		# pub_right_ankle_pitch.publish(sine_movement)
		# pub_right_ankle_roll.publish(sine_movement)
		


def gazebo_link_states():
 
	rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

	rospy.spin()


# def callback(message):
# 	link4pose = message.pose[4]
# 	link4quaternion = link4pose.orientation
# 	q = [link4quaternion.x,link4quaternion.y,link4quaternion.z,link4quaternion.w]
# 	link4orientation = euler_from_quaternion(q)

# 	end_effector_x = link4pose.position.x + sin(link4orientation[1])
# 	end_effector_y = link4pose.position.y
# 	end_effector_z = link4pose.position.z - cos(link4orientation[1])
# 	end_effector_position = [end_effector_x,end_effector_y,end_effector_z]


# 	theta_before_added_object = 21.62
# 	theta_new = atan2((link4pose.position.z-end_effector_z),(link4pose.position.x-end_effector_x))

# 	Gain = 0.4
# 	torque_before_added_object = 9.81

# 	torque_new = (4.1*torque_before_added_object/9.81)*Gain*((theta_new/pi*180) - theta_before_added_object)

# 	pub1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=100)
# 	pub2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=100)
# 	pub3 = rospy.Publisher('/rrrbot/joint3_torque_controller/command', Float64, queue_size=100)

# 	pub1.publish(2.5*torque_before_added_object)
# 	pub2.publish(1.2*torque_before_added_object)
# 	pub3.publish((4.1*torque_before_added_object/9.81)+torque_new)

# 	print theta_new/pi*180, torque_new





#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: slider_joint_torques_publisher()
	except rospy.ROSInterruptException: pass
