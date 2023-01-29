#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates

def callback(data):
    print(data.pose[1])
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()