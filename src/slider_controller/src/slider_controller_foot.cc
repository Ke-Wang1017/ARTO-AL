#include "osc_foot.h"
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
/*#include <slider_gazebo/msgs/traj_scale.msg>
*/
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
#include <vector>   
#include <cmath>

// The joint configuration (angle, velocity) should be a column Eigen vector, 
// this should be obtained from sensor data (e.g. Gazebo joint data).
VectorXd q(17);
VectorXd v(16);
VectorXd q_base(7);
VectorXd q_joint(10);
VectorXd v_base(6);
VectorXd v_joint(10);
Vector3d left_foot_pos, right_foot_pos, base_euler_angle;
// Step time
double step_time = 0.4;
int support_foot_flag;
// Remaining and current time for the current step
double remaining_time = step_time;
double next_step_time = 0.4;
double current_time = 0;
int support_foot_prev;
bool start = true;
const double slope = 0*3.14159/180.0;
double coeff = tan(slope);

// Result obtained from the MPC planner
VectorXd zmp_foothold(4);
Support whichLeg;

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

 /*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 */
Matrix3d quaternionToRotationMatrix(const VectorXd &q)
{
    double e0 = q(3); // w
    double e1 = q(0); // x
    double e2 = q(1); // y
    double e3 = q(2); // z

    Matrix3d R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
        2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
        1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
        1 - 2 * (e1 * e1 + e2 * e2);
    // R.transposeInPlace();
    return R;
}


// Joint_state callback function
// file: sensor_msgs/JointState.msg
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{   // left right
    // roll pitch slide pitch_foot, roll_foot
    q_joint << msg->position[3], msg->position[2], msg->position[4], msg->position[0], msg->position[1],
               msg->position[8], msg->position[7], msg->position[9], msg->position[5], msg->position[6];
    v_joint << msg->velocity[3], msg->velocity[2], msg->velocity[4], msg->velocity[0], msg->velocity[1],
               msg->velocity[8], msg->velocity[7], msg->velocity[9], msg->velocity[5], msg->velocity[6];
}

// Link_state callback function
// file: gazebo_msgs/Linkstate.msg
void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    q_base << msg->pose[1].position.x, msg->pose[1].position.y, (msg->pose[1].position.z),
              msg->pose[1].orientation.x, msg->pose[1].orientation.y,  msg->pose[1].orientation.z, msg->pose[1].orientation.w;
    v_base << msg->twist[1].linear.x, msg->twist[1].linear.y, msg->twist[1].linear.z,
              msg->twist[1].angular.x, msg->twist[1].angular.y, msg->twist[1].angular.z;
    // need to modify
    // cout << "q_base: \n" << q_base << endl;
//     cout << "v_base: \n" << v_base << endl;
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
//    remaining_time = msg->data[6];
//    step_time = current_time + remaining_time;
//    support_foot_flag = msg->data[7];
}

int main(int argc, char ** argv)
{
    //********************************************************************************************
    //****************************ROS node initialization*****************************************
    //********************************************************************************************
    ros::init(argc, argv, "ros_SLIDER_OSC_node");
    ros::NodeHandle n;
    // Load gazebo
    // client::setup(argc, argv);
    ros::Rate loop_rate(LOOP_RATE);

    // Publisher for ros torque control
    // ros::Publisher traj_stage_pub   = n.advertise<::traj_scale>("/time_slider_gazebo/traj_stage", 1);

    ros::Publisher left_hip_pitch_pub   = n.advertise<std_msgs::Float64>("/time_slider_gazebo/left_hip_pitch_torque_controller/command", 1);
    ros::Publisher left_hip_roll_pub    = n.advertise<std_msgs::Float64>("/time_slider_gazebo/left_hip_roll_torque_controller/command", 1);
    ros::Publisher left_hip_slide_pub   = n.advertise<std_msgs::Float64>("/time_slider_gazebo/left_hip_slide_torque_controller/command", 1);
    ros::Publisher left_ankle_roll_pub  = n.advertise<std_msgs::Float64>("/time_slider_gazebo/left_ankle_roll_torque_controller/command", 1);
    ros::Publisher left_ankle_pitch_pub = n.advertise<std_msgs::Float64>("/time_slider_gazebo/left_ankle_pitch_torque_controller/command", 1);

    ros::Publisher right_hip_pitch_pub   = n.advertise<std_msgs::Float64>("/time_slider_gazebo/right_hip_pitch_torque_controller/command", 1);
    ros::Publisher right_hip_roll_pub    = n.advertise<std_msgs::Float64>("/time_slider_gazebo/right_hip_roll_torque_controller/command", 1);
    ros::Publisher right_hip_slide_pub   = n.advertise<std_msgs::Float64>("/time_slider_gazebo/right_hip_slide_torque_controller/command", 1);
    ros::Publisher right_ankle_roll_pub  = n.advertise<std_msgs::Float64>("/time_slider_gazebo/right_ankle_roll_torque_controller/command", 1);
    ros::Publisher right_ankle_pitch_pub = n.advertise<std_msgs::Float64>("/time_slider_gazebo/right_ankle_pitch_torque_controller/command", 1);
    // Publish the data used for MPC planner
    ros::Publisher planner_input_pub = n.advertise<std_msgs::Float64MultiArray>("/time_slider_gazebo/planner_input", '1');

    // Subscribe the Gazebo joint_states topic
    ros::Subscriber sub_joint = n.subscribe<sensor_msgs::JointState>("/time_slider_gazebo/joint_states", 1, jointStateCallback);
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
    Vector3d x_hat, y_hat, z_hat;
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
    //string filePath = "/home/robin/Documents/SLIDER_ROTO/src/slider_controller";
    string filePath = ros::package::getPath("time_slider_controller");
    std::string filename_urdf = filePath + "/data/SLIDER_ROTOGAIT_FOOT_pin.urdf";
//    std::string filename_urdf = filePath + "/data/SLIDER_ROTOGAIT_pin.urdf";
    pin::Model model;
    pin::urdf::buildModel(filename_urdf,model);
    pin::Data data(model);

    // Initialize OSC controller
    OSC OSC_controller;

    // Commanded acceleration given to OSC controller
    Vector3d ddx_com_cmd, ddx_left_cmd, ddx_right_cmd, ddx_pelvis_orientation, ddx_left_ori_cmd, ddx_right_ori_cmd, left_ankle_ori_feedback, right_ankle_ori_feedback;
    Vector3d dh_ang_cmd;

    // Actual end-effector position, velocity and acceleration
    Vector3d left_pos, left_vel, left_acc, right_pos, right_vel, right_acc, left_leg;
    Vector3d left_angVel, right_angVel; 
    Matrix3d left_rot, right_rot;
    // PD parameters, negative feedback
    double Kp_com = -70; //70
    double Kd_com = -5; //5
    double Kp_orientation = -50; //70
    double Kd_orientation =  5; //5
    double Kp_left = -70; 
    double Kd_left = -5;
    double Kp_right = -70; 
    double Kd_right = -5;
    double kp_ang = -20;
    double kd_ang = -2;
    double Kp_foot_ori = -15;
    double Kd_foot_ori = 1;


    // Parameters for regulating the base orientation 
    Quaternionf base_orientation;
    double kp_base_pitch = 50; //60
    double kd_base_pitch = 3; //5
    double left_hip_pitch_feedback = 0;
    double right_hip_pitch_feedback = 0;
    double left_ankle_pitch_feedback = 0;
    double right_ankle_pitch_feedback = 0;
//    bool isFirstStep = true;
    // QP parameters
    VectorXd w(21);

    double k = 0.75/1.414; // The ground friction coefficient is 0.75
    VectorXd torque_limit(10); // The torque limit on each motor
    torque_limit << 1500, // Left hip roll
                    600, // Left hip pitch
                    600, // Left hip slide
                    300, // Left ankle pitch
                    300, // Left ankle roll
                    1500, // Right hip roll
                    600, // Right hip pitch
                    600, // Right hip slide
                    300, // Right ankle pitch
                    300; // Right ankle roll

//    ddx_pelvis_orientation << 0.0, 0.0, 0.0;

    // Optimal torque 
    VectorXd traj_stage2;
    Matrix3d R_wb;
    Matrix3d R_wb_T;
    Vector3d eps_m, r_vrp, Fd;
    Vector3d h_ang;
    Vector3d dh_ang;

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
    q_base << 0,0,0,0,0,0,1;

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
//        cout<<"THe pelvis orientation is " << base_euler_angle << endl;
        // Rotation matrix
        R_wb = quaternionToRotationMatrix(q_base.tail(4));

        // Update joint position and velocity
//        if(whichLeg != Support::S){
//            if(support_foot_flag == -1) whichLeg = Support::L;
//            if(support_foot_flag == 1) whichLeg = Support::R;
//            if(support_foot_prev != support_foot_flag){
//                counter = 0;
//                if(support_foot_flag == Support::R){ left_foot_start_x = left_pos(0); left_foot_start_y = left_pos(1);}
//                if(support_foot_flag == Support::L){ right_foot_start_x = right_pos(0); right_foot_start_y = right_pos(1);}
//            }
//        }
        if((start == true) && (counter <= update_delay)) //whichLeg == Support::S
        {
            cout<<"Start to work !!!!!!!!!!!!" << endl;
            q << 0,0,0.1,0,0,0,1,0,0,0,0,0,0,0,0,0,0;
            v << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
            base_euler_angle << 0.0, 0.0, 0.0;
            v_base << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            zmp_foothold << 0.0, -0.21, 0.0, 0.21; // in case left foot support
        }
        else
        {
             // Rotation matrix
            // R_wb = quaternionToRotationMatrix(q_base.tail(4));
            R_wb_T = R_wb.transpose();
            // cout << "Rotation matrix\n" << R_wb << endl;

            q << q_base, q_joint;
            // v << v_base, v_joint;
            v << R_wb_T * v_base.head(3),
                 R_wb_T * v_base.tail(3),
                 v_joint;
        }
//        cout<< "q base is " << q_base << endl;

        cout << "support leg is "<< whichLeg << endl;
//
        // Update date based on the current configuration of the robot
        OSC_controller.updateData(model,data,q,v,whichLeg,counter);

//         cout << "------------Left foot position------------" << endl;
        left_pos = data.oMf[model.getFrameId("Left_Foot")].translation();
        left_rot = data.oMf[model.getFrameId("Left_Foot")].rotation();
//         cout << "left foot position is " << left_pos << endl;
        // cout << "------------Left foot velocity------------" << endl;
        left_vel = pin::getFrameVelocity(model,data,model.getFrameId("Left_Foot")).linear();
        left_angVel = pin::getFrameVelocity(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL).angular();
//         cout << "left velocity" << left_vel << endl;
        // cout << "------------Left foot acceleration------------" << endl;
        left_acc = pin::getFrameAcceleration(model,data,model.getFrameId("Left_Foot")).linear();
        // cout << left_acc << endl;

//         cout << "------------Right foot position------------" << endl;
        right_pos = data.oMf[model.getFrameId("Right_Foot")].translation();
        right_rot = data.oMf[model.getFrameId("Right_Foot")].rotation();
//        right_pos(2) -= 0.02;
//         cout << "right position is " << right_pos << endl;
        // cout << "------------Right foot velocity------------" << endl;
        right_vel = pin::getFrameVelocity(model,data,model.getFrameId("Right_Foot")).linear();
        right_angVel = pin::getFrameVelocity(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL).angular();
//         cout << "right velocity " << right_vel << endl;
        // cout << "------------Right foot acceleration------------" << endl;
        right_acc = pin::getFrameAcceleration(model,data,model.getFrameId("Right_Foot")).linear();
        // cout << right_acc << endl;
//        pin::forwardKinematics(model,data,q);
        pin::computeCentroidalMomentumTimeVariation(model, data);
        // pin::computeCentroidalMomentum(model, data);

        h_ang = data.hg.angular();
        dh_ang = data.dhg.angular();
//        pin::centerOfMass(model,data,2);
//        x_hat << data.com[0][0], data.vcom[0][0], (data.vcom[0][0]-com_dot_prev(0))*LOOP_RATE;
//        y_hat << data.com[0][1], data.vcom[0][1], (data.vcom[0][1]-com_dot_prev(1))*LOOP_RATE;
        x_hat << data.com[0][0], data.vcom[0][0], data.acom[0][0];
        y_hat << data.com[0][1], data.vcom[0][1], data.acom[0][1];
       // cout<<"the com state in x "<< x_hat << endl;
//        cout<<"The com state in y " << y_hat << endl;
        z_hat << data.com[0][2], data.vcom[0][2],data.acom[0][2];

        // Publish data for the MPC planner, it will recreate every iteration
        std_msgs::Float64MultiArray planner_input;
        // Just add noise here
        planner_input.data.push_back(x_hat(0));
        planner_input.data.push_back(x_hat(1));
        planner_input.data.push_back(x_hat(2));
        planner_input.data.push_back(y_hat(0));
        planner_input.data.push_back(y_hat(1));
        planner_input.data.push_back(y_hat(2));
        // cout << "--------New iteration (" << whichLeg << ")----------" << endl;

        //***************************************************************************************
        //*****************************Run State Machine*****************************************
        //***************************************************************************************

        // Run state machine and the corresponding planner
        switch(whichLeg) 
        {
            case Support::L :

                current_time = double(counter)/LOOP_RATE;
//                remaining_time = step_time - current_time;
//                cout << "current_time: " << current_time << endl;
//                cout << "remaining_time: " << remaining_time << endl;
//                cout << "step_time: " << step_time << endl;
                planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                planner_input.data.push_back(whichLeg); // support foot
                planner_input.data.push_back(current_time); //This is the first 12 element that NEED to get passed back to the planner
                planner_input.data.push_back(z_hat(0));
                planner_input.data.push_back(z_next_foot(0));

                planner_input.data.push_back(step_time);
                planner_input.data.push_back(left_pos(2));// 13
                planner_input.data.push_back(right_pos(2)); // 14
                // planner_input.data.push_back(z_next_foot(0)); // 15
                planner_input.data.push_back(next_step_time);
                planner_input.data.push_back(x_next_foot(0));
                planner_input.data.push_back(left_pos(1));  //18
                planner_input.data.push_back(right_pos(1));
                planner_input.data.push_back(y_next_foot(0));
                planner_input.data.push_back(base_euler_angle(0));
                planner_input.data.push_back(base_euler_angle(1));
                planner_input.data.push_back(base_euler_angle(2));
//                planner_input.data.push_back((data.com[0] - x_com).norm());
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
                // x_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(0), dx_right(0), ddx_right(0), zmp_foothold(0), 0, 0); //x
                // y_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(1), dx_right(1), ddx_right(1), zmp_foothold(1), 0, 0); //y
                x_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, right_foot_start_x, 0, 0, zmp_foothold(0), 0, 0); //x
                y_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, right_foot_start_y, 0, 0, zmp_foothold(1), 0, 0); //y
                if (current_time <= (step_time/2)) // ground to peak
                     z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time/2.0-current_time), 1.0/LOOP_RATE, x_right(2), dx_right(2), ddx_right(2), step_height + coeff*(zmp_foothold(0)-foot_length/2), 0, 0); //x
                   // z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min(step_time/2, current_time), foot_height, 0.0, 0, step_height, 0.0, 0); //z
                else // peak to ground
                     z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_right(2), dx_right(2), ddx_right(2), foot_height + coeff*(zmp_foothold(0)-foot_length/2), -0.1, 0); //x
                   // z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min((current_time - step_time/2), step_time/2), step_height, 0.0, 0, foot_height, -0.1, 0); //

//                    x_right << 0.0, -0.21, 0.0;
//                    x_left << 0.0, 0.21, 0.0;
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

                // OSC weighting parameters
                w << 1, 1, 3, // ddx_com
                      0.1, 0.1, 0.1, // dh_ang
                      1, 1, 1, // ddx_left
                      1, 1, 1, // ddx_left_ori
                      1, 1, 2,  // ddx_right
                      1, 1, 1, // ddx_right_ori
                      1, 1, 1; // ddx_base_orientation

                Kp_com = -300; 
                Kd_com = -60;
                Kp_right = -500;
                Kd_right = -50;
                Kp_left = -50;
                Kd_left = -5;
                Kp_foot_ori = 35;   /// originally 50
                Kd_foot_ori = -5;
                kp_ang = -0.0;
                kd_ang = -0.0;
                Kp_orientation = 100;
                Kd_orientation = -10;

                if (current_time >= step_time - 10.0/LOOP_RATE ) //&& right_pos(2) < 0.01
                {
                    whichLeg = Support::R;
                    if(start == true) start = false;
                    counter = 0;
                    current_time = double(counter)/LOOP_RATE;
                    if(next_step_time < 1.0){
                        step_time = next_step_time;
                    } else{ step_time = 0.4;}

//                    remaining_time = step_time - double(counter)/LOOP_RATE;
                    planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                    planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                    planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(whichLeg); // support foot
                    planner_input.data.push_back(current_time); //This is the first 12 element that NEED to get passed back to the planner
                    planner_input.data.push_back(z_hat(0));
                    


                    planner_input.data.push_back(step_time);
                    planner_input.data.push_back(left_pos(2));
                    planner_input.data.push_back(right_pos(2));
                    planner_input.data.push_back(z_next_foot(0));
                    planner_input.data.push_back(next_step_time);
                    planner_input.data.push_back(x_next_foot(0));
                    planner_input.data.push_back(left_pos(1));
                    planner_input.data.push_back(right_pos(1));
                    planner_input.data.push_back(y_next_foot(0));
                    planner_input.data.push_back(base_euler_angle(0));
                    planner_input.data.push_back(base_euler_angle(1));
                    planner_input.data.push_back(base_euler_angle(2));
//                    planner_input.data.push_back((data.com[0] - x_com).norm());
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
//                    if (isFirstStep) {isFirstStep = false;}
                }

//                 cout << "Desire right foot z" << " pos: " << x_right(2) << " vel: " << dx_right(2) << endl;
//                 cout << "Actual right foot z" << " pos: " << right_pos(2) << " vel: " << right_vel(2) << endl;
//                 cout << "Desire left foot z" << " pos: " << x_left(2) << " vel: " << dx_left(2) << endl;
//                 cout << "Actual left foot z" << " pos: " << left_pos(2) << " vel: " << left_vel(2) << endl;
                // cout << "Desire right foot y" << " pos: " << x_right(1) << " vel: " << dx_right(1) << endl;
                // cout << "Actual right foot y" << " pos: " << right_pos(1) << " vel: " << right_vel(1) << endl;
//                 cout << "Planner right foot x" << " pos: " << zmp_foothold(0) << endl;
//                 cout << "Planner right foot y" << " pos: " << zmp_foothold(1) << endl;
//
                cout << "d right foot z: " << x_right(2) << endl;
                cout << "a right foot z: " << right_pos(2) << endl;
//
                cout << "d right foot y: " << x_right(1) << endl;
                cout << "a right foot y: " << right_pos(1) << endl;

                cout << "d com y: " << x_next_state(1) << endl;
                cout << "a com y: " << x_hat(1) << endl;
                cout << "remaining_time: " << step_time-current_time << endl;

//                 cout << "desired com z: " << x_com(2) << endl;
                // cout << "desired vcom y: " << dx_com(1) << endl;
                // cout << "desired acom y: " << ddx_com(1) << endl;
//                 cout << "actual com z: " << z_hat(0) << endl;
                // cout << "actual vcom y: " << y_hat(1) << endl;
                // cout << "zmp y: " << zmp_foothold(1) << endl;

                // cout << "x_left: " << x_left << endl;
                // cout << "dx_left: " << dx_left << endl;
                // cout << "ddx_left: " << ddx_left << endl;

                break;

            case Support::R :
                current_time = double(counter)/LOOP_RATE;
//                cout << "current_time: " << current_time << endl;
//                cout << "remaining_time: " << remaining_time << endl;
//                cout << "step_time: " << step_time << endl;
//                remaining_time = step_time - current_time;
                // cout << "remaining_time: " << remaining_time << endl;

                planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                planner_input.data.push_back(whichLeg); // support foot
                planner_input.data.push_back(current_time); //This is the first 12 element that NEED to get passed back to the planner
                planner_input.data.push_back(z_hat(0));


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
//                planner_input.data.push_back((data.com[0] - x_com).norm());
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

                // desired right foot trajectory
                ddx_right = Vector3d::Zero();
                dx_right = Vector3d::Zero();
                x_right = right_pos;

                // x_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(0), dx_left(0), ddx_left(0), zmp_foothold(0), 0, 0); //x
                // y_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(1), dx_left(1), ddx_left(1), zmp_foothold(1), 0, 0); //y
                x_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, left_foot_start_x, 0, 0, zmp_foothold(0), 0, 0); //x
                y_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, left_foot_start_y, 0, 0, zmp_foothold(1), 0, 0); //y

                if (current_time <= (step_time/2)) // ground to peak
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time/2.0-current_time), 1.0/LOOP_RATE, x_left(2), dx_left(2), ddx_left(2), step_height + coeff*(zmp_foothold(0)-foot_length/2), 0, 0); //x
                   // z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min(step_time/2, current_time), foot_height, 0.0, 0, step_height, 0.0, 0); //z
                else // peak to ground
                    z_next_foot = fifthOrderPolynomialInterpolation(max(1.0/LOOP_RATE, step_time-current_time), 1.0/LOOP_RATE, x_left(2), dx_left(2), ddx_left(2), foot_height + coeff*(zmp_foothold(0)-foot_length/2), -0.1, 0); //x
                   // z_next_foot = thirdOrderPolynomialInterpolation(step_time/2, min((current_time - step_time/2), step_time/2), step_height, 0.0, 0, foot_height, -0.1, 0);//

                ddx_left << x_next_foot(2), y_next_foot(2), z_next_foot(2);
                dx_left << x_next_foot(1), y_next_foot(1), z_next_foot(1);
                x_left << x_next_foot(0), y_next_foot(0), z_next_foot(0);

                // desired CoM trajectory
                ddx_com << (GRAVITY/com_height)*(x_hat(0)-right_pos(0)), (GRAVITY/com_height)*(y_hat(0)-right_pos(1)), 0.0;
                x_next_state = A_loop_rate*x_hat.head(2) + B_loop_rate*right_pos(0);
                y_next_state = A_loop_rate*y_hat.head(2) + B_loop_rate*right_pos(1);
                // cout << "CoM next state is, x: " << x_next_state(0) << " y: " << y_next_state(0)<< endl;
                dx_com << x_next_state(1), y_next_state(1), 0.0;
                x_com << x_next_state(0), y_next_state(0), slope*x_next_state(0) + com_height;

                // OSC weighting parameters
                w << 1, 1, 3, // ddx_com
                      0.1, 0.1, 0.1, // dh_ang
                      1, 1, 2, // ddx_left
                      1, 1, 1, // ddx_left_ori
                      1, 1, 1,  // ddx_right
                      1, 1, 1, // ddx_right_ori
                      1, 1, 1; // ddx_base_orientation

                Kp_com = -300; 
                Kd_com = -60;
                Kp_left = -500;
                Kd_left = -50;
                Kp_right = -50;
                Kd_right = -5;
                Kp_foot_ori = 35; /// originally 50
                Kd_foot_ori = -5;
                kp_ang = -0.0;
                kd_ang = -0.0;
                Kp_orientation = 100;
                Kd_orientation = -10;


                if (current_time >= step_time - 10.0/LOOP_RATE) // && left_pos(2) < 0.01
                {
                    whichLeg = Support::L;
                    counter = 0;
                    current_time = double(counter)/LOOP_RATE;
                    if(next_step_time < 1.0){
                        step_time = next_step_time;
                    } else{ step_time = 0.4;}
//                    remaining_time = step_time;
                    planner_input.data.push_back(left_pos(0)); // current step location in the x direction 4
                    planner_input.data.push_back(left_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(right_pos(0)); // current step location in the x direction
                    planner_input.data.push_back(right_pos(1)); // current step location in the y direction
                    planner_input.data.push_back(whichLeg); // support foot
                    planner_input.data.push_back(current_time); //This is the first 12 element that NEED to get passed back to the planner
                    planner_input.data.push_back(z_hat(0));

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
//                    planner_input.data.push_back((data.com[0] - x_com).norm());
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

//                 cout << "Desire left foot z" << " pos: " << x_left(2) << " vel: " << dx_left(2) << endl;
//                 cout << "Actual left foot z" << " pos: " << left_pos(2) << " vel: " << left_vel(2) << endl;
//                 cout << "Desire right foot z" << " pos: " << x_right(2) << " vel: " << dx_right(2) << endl;
//                 cout << "Actual right foot z" << " pos: " << right_pos(2) << " vel: " << right_vel(2) << endl;
//                 cout << "desired com z: " << x_com(2) << endl;
//                 cout << "actual com z: " << z_hat(0) << endl;
                // cout << "Desire left foot y" << " pos: " << x_left(1) << " vel: " << dx_left(1) << endl;
                // cout << "Actual left foot y" << " pos: " << left_pos(1) << " vel: " << left_vel(1) << endl;
                // cout << "Planne left foot y" << " pos: " << zmp_foothold(3) << endl;
//                 cout << "Planner left foot x" << " pos: " << zmp_foothold(0) << endl;
//                 cout << "Planner left foot y" << " pos: " << zmp_foothold(1) << endl;
//                 cout << "Desire left foot z" << " pos: " << x_left(2) << " vel: " << dx_left(2) << endl;
//                 cout << "Actual left foot z" << " pos: " << left_pos(2) << " vel: " << left_vel(2) << endl;
//                 cout << "d left foot x: " << x_next_foot(0) << endl;
//                 cout << "a left foot x: " << left_pos(0) << endl;
//
//                 cout << "d left foot y: " << y_next_foot(0) << endl;
//                 cout << "a left foot y: " << left_pos(1) << endl;
                break;

            case Support::D : // double support

                break;
        }

        //***************************************************************************************
        //*****************************Run OSC controller****************************************
        //***************************************************************************************

        // PD and feedforward controller
        ddx_com_cmd = Kp_com*(data.com[0] - x_com) + Kd_com*(data.vcom[0] - dx_com) + ddx_com;
        dh_ang_cmd = kp_ang*h_ang + kd_ang*dh_ang;
        //ddx_pelvis_orientation = R_wb*Kp_orientation*(pin::log3(R_wb.transpose())) + Kd_orientation*(v_base.tail(3));
        double beta = 0;
        Matrix3d R_d;
        R_d << cos(beta), 0, sin(beta),
                0, 1, 0,
               -sin(beta), 0, cos(beta);
        ddx_pelvis_orientation = R_wb*Kp_orientation*(pin::log3(R_wb_T*R_d)) + Kd_orientation*(v_base.tail(3));
        // ddx_pelvis_orientation = Kp_orientation*(base_euler_angle) + Kd_orientation*(v_base.segment(3,3));   
        // cout<< "orientation change is " << R_wb*(pin::log3(R_wb_T*R_d)) << endl;
        ddx_left_cmd = Kp_left*(left_pos - x_left) + Kd_left*(left_vel - dx_left); // + ddx_left
        ddx_right_cmd = Kp_right*(right_pos - x_right) + Kd_right*(right_vel - dx_right); //  + ddx_right

        Matrix3d left_rot_T = left_rot.transpose();
        Matrix3d right_rot_T = right_rot.transpose();

        Matrix3d R_f;
        R_f << cos(-slope), 0, sin(-slope),
        0, 1, 0,
       -sin(-slope), 0, cos(-slope);

        ddx_left_ori_cmd = left_rot*(Kp_foot_ori*pin::log3(left_rot_T*R_f) + Kd_foot_ori*left_angVel);
        ddx_right_ori_cmd = right_rot*(Kp_foot_ori*pin::log3(right_rot_T*R_f) + Kd_foot_ori*right_angVel);
//         cout << "******************************" << endl;
//         cout<< "REF com is " << x_com << endl;
        cout<< "Real COM is " << data.com[0] << endl;
        cout << "COM position difference: \n" << data.com[0] - x_com << endl;
//         cout << "******************************" << endl;
//         cout << "COM velocity difference: \n" << data.vcom[0] - dx_com << endl;
        cout << "******************************" << endl;
        cout << "Left foot position difference: \n" << left_pos - x_left << endl;
//         cout << "Des Left foot position: \n" << x_left << endl;
//         cout << "Rea Left foot position: \n" << x_left << endl;
//         cout << "******************************" << endl;
//         cout << "Left foot velocity difference: \n" << left_vel - dx_left << endl;
        cout << "******************************" << endl;
        cout << "Right foot position difference: \n" << right_pos - x_right << endl;
//         cout << "Des Right foot position difference: \n" << x_right << endl;
//         cout << "******************************" << endl;
//         cout << "Right foot velocity difference: \n" << right_vel - dx_right << endl;
//         cout << "******************************" << endl;

        // Solve for the optimal torque
        traj_stage2 = OSC_controller.solveQP(ddx_com_cmd, dh_ang_cmd, ddx_left_cmd, ddx_right_cmd, ddx_left_ori_cmd, ddx_right_ori_cmd, ddx_pelvis_orientation, w, k, torque_limit, counter);
//        cout<< "output Joint torque is " << traj_stage2 << endl;
/*        traj_stage_pub.publish(getROSmsg(traj_stage2));*/


        left_hip_roll_pub.publish(getROSmsg(traj_stage2[0]));
        left_hip_pitch_pub.publish(getROSmsg(traj_stage2[1])); // + left_hip_pitch_feedback

        left_hip_slide_pub.publish(getROSmsg(traj_stage2[2]));
        left_ankle_pitch_pub.publish(getROSmsg(traj_stage2[3]));
        left_ankle_roll_pub.publish(getROSmsg(traj_stage2[4]));
        
        right_hip_roll_pub.publish(getROSmsg(traj_stage2[5]));
        right_hip_pitch_pub.publish(getROSmsg(traj_stage2[6])); //  + right_hip_pitch_feedback
        
        right_hip_slide_pub.publish(getROSmsg(traj_stage2[7]));
        right_ankle_pitch_pub.publish(getROSmsg(traj_stage2[8]));
        right_ankle_roll_pub.publish(getROSmsg(traj_stage2[9]));
//        cout<<"output torque is " << traj_stage2 << endl;
//        left_hip_roll_pub.publish(getROSmsg(0.0));
//        left_hip_pitch_pub.publish(getROSmsg(0.0)); // + left_hip_pitch_feedback
//        left_hip_slide_pub.publish(getROSmsg(0.0));
//
//        right_hip_roll_pub.publish(getROSmsg(0.0));
//        right_hip_pitch_pub.publish(getROSmsg(0.0)); //  + right_hip_pitch_feedback
//        right_hip_slide_pub.publish(getROSmsg(0.0));

        counter++;
//        cout<<"counter is " << counter << endl;
        support_foot_prev = support_foot_flag;

        ros::spinOnce();

        loop_rate.sleep();
    }
	return 0;
}