import rospy
import rospkg
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from gazebo_msgs.srv import ApplyBodyWrench
import numpy as np

t_start = 0.0
apply_time = 0.01  # in seconds
apply_time2 = 0.5 # in seconds
force = 80
force2 = 0
angle = 0  # degrees ANTIclockwise from x direction looking from above
angle2 = 90   # degrees clockwise from x direction looking from above

def callback(msg):
    global t_start
    # just using to find when simulation starts
    now = rospy.Time.now()
    if not t_start > 0.0:
        t_start = now.secs + now.nsecs/1e9

def main():
    rospy.init_node("torque_node", disable_signals=True)
    rospy.Subscriber('/time_slider_gazebo/planner_input', Float64MultiArray, callback)
    # rospy.Subscriber('/slider_gazebo/planner_input', Float64MultiArray, callback)
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    wrench = Wrench()
    wrench.force.x = force * np.cos(np.deg2rad(angle))
    wrench.force.y = force * np.sin(np.deg2rad(angle))
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0

    wrench2 = Wrench()
    wrench2.force.x = force2 * np.cos(np.deg2rad(angle2))
    wrench2.force.y = force2 * np.sin(np.deg2rad(angle2))
    wrench2.force.z = 0
    wrench2.torque.x = 0
    wrench2.torque.y = 0
    wrench2.torque.z = 0

    r = rospy.Rate(100)
    first_push = True
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        print('time is ', now)
        if t_start > 0.0:
            t_now = now.secs + now.nsecs/1e9
            print(t_now - t_start)
            if t_now - t_start > apply_time and first_push:
                apply_body_wrench("time_slider_gazebo::Right_Roll_Pitch_Link",  # body name
                # apply_body_wrench("time_slider_gazebo::pelvis",  # body name
                                  "",  # reference frame
                                  None,  # reference point
                                  wrench,  # wrench
                                  rospy.Time.from_sec(0),  # start time
                                  rospy.Duration.from_sec(0.1))  # duration
                first_push = False
            # if t_now - t_start > apply_time2:
            #     apply_body_wrench("time_slider_gazebo::base_apply",  # body name
            #                       "",  # reference frame
            #                       None,  # reference point
            #                       wrench2,  # wrench
            #                       rospy.Time.from_sec(0),  # start time
            #                       rospy.Duration.from_sec(0.1))  # duration
                rospy.signal_shutdown("applied force, shutting down node")
        r.sleep()


if __name__ == "__main__":
    main()
