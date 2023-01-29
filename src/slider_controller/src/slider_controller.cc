#include "osc.h"
#include "gen_trajectory.h"
#include "define.h"
#include "LIPM.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <chrono>
#include <cmath>

// The joint configuration (angle, velocity) should be a column Eigen vector, 
// this should be obtained from sensor data (e.g. Gazebo joint data).
VectorXd q(13);
VectorXd v(12); // 6 joints,3 linear velocity 3 angular velocity
VectorXd v_pre(12);
VectorXd a(12);
VectorXd q_base(7);
VectorXd q_joint(6);
VectorXd v_base(6);
VectorXd v_joint(6);
Vector3d base_euler_angle; // left_foot_pos, right_foot_pos,
const double alpha = 10.0*3.14159/180.0;
//const double x_offset = 0.0625;//
// Step time
double step_time = 0.4;
int support_foot_flag;
// Remaining and current time for the current step
double remaining_time = step_time;
double next_step_time = 0.4;
double current_time = 0;
int support_foot_prev;
bool start = true;
// Result obtained from the MPC planner
VectorXd zmp_foothold(4);
Support whichLeg;

double slope = sin(10*PI/180);

std_msgs::Float64 getROSmsg(double torque)
{
  std_msgs::Float64 msg;

  msg.data = torque;
  return msg;
}

struct EulerAngles
{
    double roll, pitch, yaw;
};

Vector3d ToEulerAngles(Quaternionf q)
{
    EulerAngles angles;
    Vector3d euler_angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = atan(sinr_cosp/cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(PI/2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
    angles.yaw = atan(siny_cosp/cosy_cosp);
    euler_angles << angles.roll, angles.pitch, angles.yaw;

    return euler_angles;
}

// Joint_state callback function
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{   //left pitch roll slide
    q_joint << msg->position[1], msg->position[0], msg->position[2], msg->position[4], msg->position[3], msg->position[5];
    v_joint << msg->velocity[1], msg->velocity[0], msg->velocity[2], msg->velocity[4], msg->velocity[3], msg->velocity[5];

}

// Link_state callback function
void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    q_base << msg->pose[1].position.x, msg->pose[1].position.y, (msg->pose[1].position.z),
              msg->pose[1].orientation.x, msg->pose[1].orientation.y,  msg->pose[1].orientation.z, msg->pose[1].orientation.w;
    v_base << msg->twist[1].linear.x, msg->twist[1].linear.y, msg->twist[1].linear.z,
              msg->twist[1].angular.x, msg->twist[1].angular.y, msg->twist[1].angular.z;
}

// Callback function for receiving the planner outputs
void footstepPlanCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    zmp_foothold(0) = msg->data[0];
    zmp_foothold(1) = msg->data[1];
    zmp_foothold(2) = msg->data[2];
    zmp_foothold(3) = msg->data[3];
    remaining_time = msg->data[4];
    next_step_time = msg->data[5];
    support_foot_flag = msg->data[6];
    step_time = current_time + remaining_time;
}

int main(int argc, char ** argv)
{
    //********************************************************************************************
    //****************************ROS node initialization*****************************************
    //********************************************************************************************
    ros::init(argc, argv, "ros_SLIDER_OSC_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);

    // Publisher for ros torque control
    ros::Publisher left_hip_pitch_pub   = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/left_hip_pitch_torque_controller/command", 1);
    ros::Publisher left_hip_roll_pub    = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/left_hip_roll_torque_controller/command", 1);
    ros::Publisher left_hip_slide_pub   = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/left_hip_slide_torque_controller/command", 1);
    ros::Publisher right_hip_pitch_pub   = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/right_hip_pitch_torque_controller/command", 1);
    ros::Publisher right_hip_roll_pub    = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/right_hip_roll_torque_controller/command", 1);
    ros::Publisher right_hip_slide_pub   = n.advertise<std_msgs::Float64>("/SLIDER_ROTOGAIT/right_hip_slide_torque_controller/command", 1);
    // Publish the data used for MPC planner
    ros::Publisher planner_input_pub = n.advertise<std_msgs::Float64MultiArray>("/time_slider_gazebo/planner_input", '1');

    // Subscribe the Gazebo joint_states topic
    ros::Subscriber sub_joint = n.subscribe<sensor_msgs::JointState>("/SLIDER_ROTOGAIT/joint_states", 1, jointStateCallback);
    ros::Subscriber sub_link = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, linkStateCallback);

    // Subscribe the data from the MPC planner
    ros::Subscriber sub_planner_output = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/footstep_plan", 1, footstepPlanCallback);

    //********************************************************************************************
    //****************************Prepare for the planner I/O*************************************
    //********************************************************************************************
    // Linver Inverted Pendulum model
    Matrix2d A_loop_rate;
    Vector2d B_loop_rate;
    genLIPM(A_loop_rate, B_loop_rate, (1.0/LOOP_RATE));

    // Current support foot
    whichLeg = Support::L;

    // Desired first stage trajectories
    Vector3d ddx_com, dx_com, x_com, ddx_left, dx_left, x_left, ddx_right, dx_right, x_right;

    // Swing foot start position
    double right_foot_start_x = 0;
    double right_foot_start_y = 0;
    double left_foot_start_x = 0;
    double left_foot_start_y = 0;
    double right_foot_start_xVel = 0;
    double right_foot_start_yVel = 0;
    double right_foot_start_zVel = 0;
    double left_foot_start_xVel = 0;
    double left_foot_start_yVel = 0;
    double left_foot_start_zVel = 0;

    // Estimated CoM state in the x and y direction
    Vector3d x_hat, y_hat; // , z_hat
    Vector2d com_dot_prev;

    // Next CoM state to be tracked by the OSC controller
    Vector2d x_next_state, y_next_state;

    // Next foot trajectory (pos, vel, acc) in the x, y and z direction
    Vector3d x_next_foot, y_next_foot, z_next_foot; //
    Vector3d y_next_com;

    //********************************************************************************************
    //*********************************OSC initialization*****************************************
    //********************************************************************************************
    int counter = 0;  

    // Build kinematics and dynamics model from urdf
    // I need to change the path, better use relative path!
    string filePath = "/home/robin/Documents/SLIDER_ROTO/src/slider_controller";
    // string filePath = ros::package::getPath("time_slider_controller");
    std::string filename_urdf = filePath + "/data/SLIDER_ROTOGAIT_pin.urdf";
//    std::string filename_urdf = filePath + "/data/SLIDER_ROTOGAIT_pin.urdf";
    pin::Model model;
    pin::urdf::buildModel(filename_urdf,model);
    pin::Data data(model);

    // Initialize OSC controller
    OSC OSC_controller;

    // Commanded acceleration given to OSC controller
    Vector3d ddx_com_cmd, ddx_left_cmd, ddx_right_cmd, ddx_pelvis_orientation;

    // Actual end-effector position, velocity and acceleration
    Vector3d left_pos, left_vel, left_acc, right_pos, right_vel, right_acc, left_leg;

    // PD parameters, negative feedback
    double Kp_com = -70; //70
    double Kd_com = -5; //5
    double Kp_orientation = -70; //70
    double Kd_orientation = -5; //5
    double Kp_left = -70; 
    double Kd_left = -5;
    double Kp_right = -70; 
    double Kd_right = -5;

    // Parameters for regulating the base orientation 
    Quaternionf base_orientation;
    double kp_base_pitch = 50; //60
    double kd_base_pitch = 3; //5
    double left_hip_pitch_feedback = 0;
    double right_hip_pitch_feedback = 0;
    // QP parameters
    VectorXd w(12);

    double k = 0.75/1.414; // The ground friction coefficient is 0.75
    VectorXd torque_limit(6); // The torque limit on each motor
    torque_limit << 350, // Left hip roll
                    250, // Left hip pitch
                    300, // Left hip slide
                    350, // Right hip roll
                    250, // Right hip pitch
                    300; // Right hip slide

    // Optimal torque 
    VectorXd traj_stage2;
    support_foot_prev = Support::L;
    // Give the initial start at first
    if (whichLeg == Support::L){ right_foot_start_x = 0.0; right_foot_start_y = -0.21; }
    if (whichLeg == Support::R){ left_foot_start_x = 0.0; left_foot_start_y = 0.21; }

    x_right << 0.0, -0.21, 0.0;
    dx_right << 0.0, 0.0, 0.0;
    ddx_right << 0.0, 0.0, 0.0;
    x_left << 0.0, 0.21, 0.0;
    dx_left << 0.0, 0.0, 0.0;
    ddx_left << 0.0, 0.0, 0.0;
    com_dot_prev << 0.0, 0.0;
    //***************************************************************************************
    //****************************Online Calculations****************************************
    //***************************************************************************************

    while (ros::ok())
    {
        // get base orientation represented in Euler angles
        base_orientation.x() = q_base[3];
        base_orientation.y() = q_base[4];
        base_orientation.z() = q_base[5];
        base_orientation.w() = q_base[6];
        base_euler_angle = ToEulerAngles(base_orientation);

        if((start == true) && (counter <= update_delay)) //whichLeg == Support::S
        {
            cout<<"Start to work !!!!!!!!!!!!" << endl;
            q << 0,0,0,0,0,0,1,0,0,0,0,0,0;
            v << 0,0,0,0,0,0,0,0,0,0,0,0;
            base_euler_angle << 0.0, 0.0, 0.0;
            v_base << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            zmp_foothold << 0.0, -0.15, 0.0, 0.15; // in case left foot support
        }
        else
        {
            q << q_base, q_joint;
            v << v_base, v_joint;
        }

        cout << "support leg is "<< whichLeg << endl;
//
        // Update data based on the current configuration of the robot
        OSC_controller.updateData(model,data,q,v,whichLeg,counter);


        left_pos = data.oMf[model.getFrameId("Left_Foot_A")].translation();

        left_vel = pin::getFrameVelocity(model,data,model.getFrameId("Left_Foot_A")).linear();

        left_acc = pin::getFrameAcceleration(model,data,model.getFrameId("Left_Foot_A")).linear();

        right_pos = data.oMf[model.getFrameId("Right_Foot_A")].translation();

        right_vel = pin::getFrameVelocity(model,data,model.getFrameId("Right_Foot_A")).linear();

        right_acc = pin::getFrameAcceleration(model,data,model.getFrameId("Right_Foot_A")).linear();

        a = (v-v_pre)*LOOP_RATE;
        pin::centerOfMass(model,data,q,v, a);
        v_pre = v;

        x_hat << data.com[0][0], data.vcom[0][0], data.acom[0][0];
        y_hat << data.com[0][1], data.vcom[0][1], data.acom[0][1];

        std_msgs::Float64MultiArray planner_input;
        planner_input.data.push_back(x_hat(0));
        planner_input.data.push_back(x_hat(1));
        planner_input.data.push_back(x_hat(2));
        planner_input.data.push_back(y_hat(0));
        planner_input.data.push_back(y_hat(1));
        planner_input.data.push_back(y_hat(2));

        //***************************************************************************************
        //*****************************Run State Machine*****************************************
        //***************************************************************************************

        // Run state machine and the corresponding planner
        switch(whichLeg) 
        {
            case Support::L :

                current_time = double(counter)/LOOP_RATE;

                planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                planner_input.data.push_back(whichLeg); // support foot
                planner_input.data.push_back(current_time);
                planner_input.data.push_back(step_time);
                planner_input.data.push_back(left_pos(2));// 13
                planner_input.data.push_back(right_pos(2)); // 14
                planner_input.data.push_back(z_next_foot(0)); // 15
                planner_input.data.push_back(next_step_time);
                planner_input.data.push_back(left_pos(0));  //15
                planner_input.data.push_back(right_pos(0));
                planner_input.data.push_back(x_next_foot(0));
                planner_input.data.push_back(left_pos(1));  //18
                planner_input.data.push_back(right_pos(1));
                planner_input.data.push_back(y_next_foot(0));
                planner_input.data.push_back(base_euler_angle(0));
                planner_input.data.push_back(base_euler_angle(1));
                planner_input.data.push_back(base_euler_angle(2));
                planner_input.data.push_back(data.com[0][2]-x_com(2));//22
                planner_input.data.push_back((data.vcom[0] - dx_com).norm());
                planner_input.data.push_back(ddx_com.norm());
                planner_input.data.push_back((left_pos - x_left).norm());
                planner_input.data.push_back((left_vel - dx_left).norm());
                planner_input.data.push_back(ddx_left.norm());
                planner_input.data.push_back((right_pos - x_right).norm());
                planner_input.data.push_back((right_vel - dx_right).norm());
                planner_input.data.push_back(ddx_right.norm());
                planner_input.data.push_back((base_euler_angle).norm());
                planner_input.data.push_back((v_base.segment(3,3)).norm()); // 32
                planner_input_pub.publish(planner_input);

                // desired left foot trajectory
                ddx_left = Vector3d::Zero();
                dx_left = Vector3d::Zero();
                x_left = left_pos;
                x_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(0), dx_right(0), ddx_right(0), zmp_foothold(0), 0, 0); //x
                y_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(1), dx_right(1), ddx_right(1), zmp_foothold(1), 0, 0); //y
                
                if (current_time <= (step_time/2)) // ground to peay
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time/2.0-current_time), 1.0/LOOP_RATE, x_right(2), dx_right(2), ddx_right(2), step_height + slope*zmp_foothold(0) - 0.1, 0, 0); //x
//                    z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min(step_time/2, current_time), foot_height, 0.0, 0, step_height, 0.0, 0); //z
                else // peak to ground
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(2), dx_right(2), ddx_right(2), foot_height + slope*zmp_foothold(0) - 0.1, -0.05, 0); //x
//                    z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min((current_time - step_time/2), step_time/2), step_height, 0.0, 0, foot_height, 0, 0); //

                ddx_right << x_next_foot(2), y_next_foot(2), z_next_foot(2);
                dx_right << x_next_foot(1), y_next_foot(1), z_next_foot(1);
                x_right << x_next_foot(0), y_next_foot(0), z_next_foot(0);

                // desired CoM trajectory
                ddx_com << (GRAVITY/com_height)*(x_hat(0)-left_pos(0)), (GRAVITY/com_height)*(y_hat(0)-left_pos(1)), 0.0;//
                x_next_state = A_loop_rate*x_hat.head(2) + B_loop_rate*left_pos(0);
                y_next_state = A_loop_rate*y_hat.head(2) + B_loop_rate*left_pos(1);
//                cout << "CoM next state is, x: " << x_next_state(0) << " y: " << y_next_state(0)<< endl;
                dx_com << x_next_state(1), y_next_state(1), 0.0;
                x_com << x_next_state(0), y_next_state(0), slope*x_next_state(0) + com_height;
                cout << "COM next state is " << x_com << endl;

                // OSC weighting parameters
                w << 1, 1, 1, // Weights on com acceleration
                     1, 1, 1, // Weights on left foot acceleration
                     10, 10, 10, // Weights on right foot acceleration
                     1, 1, 1; // Weights on pelvis orientation acceleration

                Kp_com = -300;
                Kd_com = -30;
                Kp_orientation = -20;
                Kd_orientation = -2;
                Kp_right = -400.0;
                Kd_right = -40.0;
                Kp_left = -30.0;
                Kd_left = -3;

                if (current_time >= step_time - 1.0/LOOP_RATE && right_pos(2) < 0.01) //
                {
                    whichLeg = Support::R;
                    if(start == true) start = false;
                    counter = 0;
                    current_time = double(counter)/LOOP_RATE;
                    if(next_step_time < 1.0){
                        step_time = next_step_time;
                    } else{ step_time = 0.4;}

                    planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                    planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                    planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(whichLeg); // support foot
                    planner_input.data.push_back(current_time);
                    planner_input.data.push_back(step_time);
                    planner_input.data.push_back(left_pos(2));
                    planner_input.data.push_back(right_pos(2));
                    planner_input.data.push_back(z_next_foot(0));
                    planner_input.data.push_back(next_step_time);
                    planner_input.data.push_back(left_pos(0));
                    planner_input.data.push_back(right_pos(0));
                    planner_input.data.push_back(x_next_foot(0));
                    planner_input.data.push_back(left_pos(1));
                    planner_input.data.push_back(right_pos(1));
                    planner_input.data.push_back(y_next_foot(0));
                    planner_input.data.push_back(base_euler_angle(0));
                    planner_input.data.push_back(base_euler_angle(1));
                    planner_input.data.push_back(base_euler_angle(2));
                    planner_input.data.push_back(data.com[0][2]-x_com(2));
                    planner_input.data.push_back((data.vcom[0] - dx_com).norm());
                    planner_input.data.push_back(ddx_com.norm());
                    planner_input.data.push_back((left_pos - x_left).norm());
                    planner_input.data.push_back((left_vel - dx_left).norm());
                    planner_input.data.push_back(ddx_left.norm());
                    planner_input.data.push_back((right_pos - x_right).norm());
                    planner_input.data.push_back((right_vel - dx_right).norm());
                    planner_input.data.push_back(ddx_right.norm());
                    planner_input.data.push_back((base_euler_angle).norm());
                    planner_input.data.push_back((v_base.segment(3,3)).norm()); // 32
                    planner_input_pub.publish(planner_input);

                    left_foot_start_x = left_pos(0);
                    left_foot_start_y = left_pos(1);
                    left_foot_start_xVel = left_vel(0);
                    left_foot_start_yVel = left_vel(1);
                    left_foot_start_zVel = left_vel(2);
                    // in case the footstep planner hasn't updated, use future step for next step
                    zmp_foothold(0) = zmp_foothold(2);
                    zmp_foothold(1) = zmp_foothold(3);
                    cout << "reach here, swith to Right foot !!!!!!!!!!!!!!!!!!!!!" << endl;

                }

                break;

            case Support::R :
                current_time = double(counter)/LOOP_RATE;

                planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                planner_input.data.push_back(whichLeg); // support foot
                planner_input.data.push_back(current_time);
                planner_input.data.push_back(left_pos(2));
                planner_input.data.push_back(right_pos(2));
                planner_input.data.push_back(z_next_foot(0));
                planner_input.data.push_back(next_step_time);
                planner_input.data.push_back(left_pos(0));
                planner_input.data.push_back(right_pos(0));
                planner_input.data.push_back(x_next_foot(0));
                planner_input.data.push_back(left_pos(1));
                planner_input.data.push_back(right_pos(1));
                planner_input.data.push_back(y_next_foot(0));
                planner_input.data.push_back(base_euler_angle(0));
                planner_input.data.push_back(base_euler_angle(1));
                planner_input.data.push_back(base_euler_angle(2));
                planner_input.data.push_back(data.com[0][2]-x_com(2));
                planner_input.data.push_back((data.vcom[0] - dx_com).norm());
                planner_input.data.push_back(ddx_com.norm());
                planner_input.data.push_back((left_pos - x_left).norm());
                planner_input.data.push_back((left_vel - dx_left).norm());
                planner_input.data.push_back(ddx_left.norm());
                planner_input.data.push_back((right_pos - x_right).norm());
                planner_input.data.push_back((right_vel - dx_right).norm());
                planner_input.data.push_back(ddx_right.norm());
                planner_input.data.push_back((base_euler_angle).norm());
                planner_input.data.push_back((v_base.segment(3,3)).norm()); // 32
                planner_input_pub.publish(planner_input);

                // desired CoM trajectory
                ddx_com << (GRAVITY/com_height)*(x_hat(0)-right_pos(0)), (GRAVITY/com_height)*(y_hat(0)-right_pos(1)), 0.0;
                x_next_state = A_loop_rate*x_hat.head(2) + B_loop_rate*right_pos(0);
                y_next_state = A_loop_rate*y_hat.head(2) + B_loop_rate*right_pos(1);
                // cout << "CoM next state is, x: " << x_next_state(0) << " y: " << y_next_state(0)<< endl;
                dx_com << x_next_state(1), y_next_state(1), 0.0;
                x_com << x_next_state(0), y_next_state(0), slope*x_next_state(0) + com_height;

                                // desired right foot trajectory
                ddx_right = Vector3d::Zero();
                dx_right = Vector3d::Zero();
                x_right = right_pos;

                x_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(0), dx_left(0), ddx_left(0), zmp_foothold(0), 0, 0); //x
                y_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(1), dx_left(1), ddx_left(1), zmp_foothold(1), 0, 0); //y

                if (current_time <= (step_time/2)) // ground to peak
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time/2.0-current_time), 1.0/LOOP_RATE, x_left(2), dx_left(2), ddx_left(2), step_height + slope*zmp_foothold(0) - 0.1, 0, 0); //x
//                    z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min(step_time/2, current_time), foot_height, 0.0, 0, step_height, 0.0, 0); //z
                else // peak to ground
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(2), dx_left(2), ddx_left(2), foot_height + slope*zmp_foothold(0) - 0.1, -0.05, 0); //x
//                    z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min((current_time - step_time/2), step_time/2), step_height, 0.0, 0, foot_height, 0, 0);//

                ddx_left << x_next_foot(2), y_next_foot(2), z_next_foot(2);
                dx_left << x_next_foot(1), y_next_foot(1), z_next_foot(1);
                x_left << x_next_foot(0), y_next_foot(0), z_next_foot(0);

                // OSC weighting parameters
                w << 1, 1, 1, // Weights on com acceleration
                     10, 10, 10, // Weights on left foot acceleration
                     1, 1, 1, // Weights on right foot acceleration
                     1, 1, 1; // Weights on pelvis orientation acceleration

                Kp_com = -300;
                Kd_com = -30;
                Kp_orientation = -20;
                Kd_orientation = -2;
                Kp_left = -400;
                Kd_left = -40;
                Kp_right = -30;
                Kd_right = -3;


                if (current_time >= step_time - 1.0/LOOP_RATE  && left_pos(2) < 0.01) //
                {
                    whichLeg = Support::L;
                    counter = 0;
                    current_time = double(counter)/LOOP_RATE;
                    if(next_step_time < 1.0){
                        step_time = next_step_time;
                    } else{ step_time = 0.4;}
                    planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                    planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                    planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(whichLeg); // support foot
                    planner_input.data.push_back(current_time);
                    planner_input.data.push_back(left_pos(2));
                    planner_input.data.push_back(right_pos(2));
                    planner_input.data.push_back(z_next_foot(0));
                    planner_input.data.push_back(left_pos(0));
                    planner_input.data.push_back(right_pos(0));
                    planner_input.data.push_back(x_next_foot(0));
                    planner_input.data.push_back(left_pos(1));
                    planner_input.data.push_back(right_pos(1));
                    planner_input.data.push_back(y_next_foot(0));
                    planner_input.data.push_back(base_euler_angle(0));
                    planner_input.data.push_back(base_euler_angle(1));
                    planner_input.data.push_back(base_euler_angle(2));
                    planner_input.data.push_back(data.com[0][2]-x_com(2));
                    planner_input.data.push_back((data.vcom[0] - dx_com).norm());
                    planner_input.data.push_back(ddx_com.norm());
                    planner_input.data.push_back((left_pos - x_left).norm());
                    planner_input.data.push_back((left_vel - dx_left).norm());
                    planner_input.data.push_back(ddx_left.norm());
                    planner_input.data.push_back((right_pos - x_right).norm());
                    planner_input.data.push_back((right_vel - dx_right).norm());
                    planner_input.data.push_back(ddx_right.norm());
                    planner_input.data.push_back((base_euler_angle).norm());
                    planner_input.data.push_back((v_base.segment(3,3)).norm()); // 32
                    planner_input_pub.publish(planner_input);
                    right_foot_start_x = right_pos(0);
                    right_foot_start_y = right_pos(1);
                    right_foot_start_xVel = right_vel(0);
                    right_foot_start_yVel = right_vel(1);
                    right_foot_start_zVel = right_vel(2);
                    // in case the footstep planner hasn't updated, use future step for next step
                    zmp_foothold(0) = zmp_foothold(2);
                    zmp_foothold(1) = zmp_foothold(3);
                    cout << "reach here, swith to left foot !!!!!!!!!!!!!!!!!!!!!" << endl;
                }

                break;

            case Support::D : // double support (not implemented yet)

                break;
        }

        //***************************************************************************************
        //*****************************Run OSC controller****************************************
        //***************************************************************************************

        // PD and feedforward controller
        ddx_com_cmd = Kp_com*(data.com[0] - x_com) + Kd_com*(data.vcom[0] - dx_com) + ddx_com;
        ddx_pelvis_orientation = Kp_orientation*(base_euler_angle) + Kd_orientation*(v_base.segment(3,3));
        ddx_left_cmd = Kp_left*(left_pos - x_left) + Kd_left*(left_vel - dx_left)+ ddx_left;
        ddx_right_cmd = Kp_right*(right_pos - x_right) + Kd_right*(right_vel - dx_right) + ddx_right;

        // Solve for the optimal torque
        traj_stage2 = OSC_controller.solveQP(ddx_com_cmd, ddx_left_cmd, ddx_right_cmd, ddx_pelvis_orientation, w, k, torque_limit, counter);
        left_hip_roll_pub.publish(getROSmsg(traj_stage2[0]));
        left_hip_pitch_pub.publish(getROSmsg(traj_stage2[1])); // + left_hip_pitch_feedback
        left_hip_slide_pub.publish(getROSmsg(traj_stage2[2]));

        right_hip_roll_pub.publish(getROSmsg(traj_stage2[3]));
        right_hip_pitch_pub.publish(getROSmsg(traj_stage2[4])); //  + right_hip_pitch_feedback
        right_hip_slide_pub.publish(getROSmsg(traj_stage2[5]));
        counter++;

        support_foot_prev = support_foot_flag;

        ros::spinOnce();

        loop_rate.sleep();
    }
	return 0;
}
