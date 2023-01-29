import rospy
import rospkg
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from gazebo_msgs.srv import ApplyBodyWrench
import numpy as np

t_start = 0.0
apply_time = 0.05  # 2.0 in seconds
velocity = 0.1
angle = 180 # degrees clockwise from x direction looking from above


def callback(msg):
    global t_start
    # just using to find when simulation starts
    now = rospy.Time.now()
    if not t_start > 0.0:
        t_start = now.secs + now.nsecs / 1e9

def main():
    rospy.init_node("velocity_node", disable_signals=True)
    rospy.Subscriber('/time_slider_gazebo/planner_input', Float64MultiArray, callback)
    # rospy.Subscriber('/slider_gazebo/planner_input', Float64MultiArray, callback)
    # pub = rospy.Publisher('/slider_gazebo/reference_velocity', Float64MultiArray, queue_size=1)
    pub = rospy.Publisher('/time_slider_gazebo/reference_velocity', Float64MultiArray, queue_size=1)

    x_vel = velocity * np.cos(np.deg2rad(angle))
    y_vel = velocity * np.sin(np.deg2rad(angle))

    ref_vel = Float64MultiArray()
    ref_vel.layout.dim = [MultiArrayDimension('', 2, 1)]
    ref_vel.data = [x_vel, y_vel]

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t_now = now.secs + now.nsecs/1e9
        if t_start > 0.0:
            if t_now - t_start > apply_time:
                pub.publish(ref_vel)
                rospy.signal_shutdown("applied velocity, shutting down node")
        r.sleep()

if __name__ == "__main__":
    main()
