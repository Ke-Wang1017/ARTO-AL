///lmax changed--> this has Qxy and walks forever

#include <iostream>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

#include <fstream>

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "analytical_gradients.h"

using namespace Eigen;
using namespace std;

const double m = 11.5;    // body mass
const double m_s = 0.3;   // swing leg mass
const double L = 1.2;     // leg length
const double h = 0.7;     // body height
const double g = 9.81;    // gravity
const double omega = sqrt(g / h);  // omega

const double mu_step_size = 0.02;

int flag = 0;
int support_foot_flag = 1;
Vector2d x_dot_ref(0.0, 0.0);     // m/s

// Parameters subscribed from the slider_controller node
// These 3 are the current x and y position-velocity-acceleration
Vector2d x_0(0.0, 0.0);	
Vector2d x_dot_0(0.0, 0.0);	
Vector2d x_dot_dot_0(0.0, 0.0);	
Vector3d dt_s(0.4, 0.4, 0.4);// This is the current foostep timing solution coming from the Python file (which uses the footstep positions u)
Vector3d dt_s_backup(0.4, 0.4, 0.4);

Matrix<double,2,3> u_s((Matrix<double,2,3>() << 0.0, -1.93340069e-02, 1.22170610e-01, -0.21, 0.21, -0.21).finished());
// Matrix<double,2,3> u_s_backup((Matrix<double,2,3>() << 0.0, -1.93340069e-02, 1.22170610e-01, -0.21, 0.21, -0.21).finished());


Vector3d t_s_rk4; //time received from rk4 (or casadi)
Matrix<double,2,3> u_s_rk4;

double Qx = 1.0;
double Qy = 1.0;

double left_x = 0.0;
double left_y = 0.21;
double right_x = 0.0;
double right_y = -0.21;
int foot_flag = 1;
// penalty weights
const int n_weights = 12;

//weight that acts as lambda in the lagrangian equations
VectorXd w; 
VectorXd w_1;
VectorXd w_2;
VectorXd w_backup3;
VectorXd w_com;

const int n_dynamic_weights = 12;
//VectorXd dyn_w;
VectorXd w_update;
MatrixXd Jacobian;
MatrixXd HessianInv;

const double max_iter = 200;
int n_repeats = 1;
bool rk4_received = false; //This will tell us when we receive the first solution (u and dt) from rk4
bool w_received = false;

// constraints
// line up with Python
const double lmax = pow(0.6*sqrt(pow(L, 2.0) - pow(h, 2.0)),2);  // lmax^2
const double rfoot = 0.2;   // No foot cross distance
const double dt0_min = 0.2; // time limit
const double dt_min = 0.4;
const double dt_max = 0.6;

double current_time = 0.0;

double dxdot_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx, int kt);
double dx_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx, int kt);
double dxdot_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx, int ku);
double dx_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx, int ku);




double J(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
    double J = 0.0;
    for(int i=1; i<u_s.size()/2+1; i++){   ////Here we are summing the Js over the time horizon you are optimizing for
        J += pow(xdot_s(0, i)-x_dot_ref[0],2)*Qx + pow(xdot_s(1, i)-x_dot_ref[1],2)*Qy; //J = J + (xdot_s-x_dot_ref)^2 + (xdot_s-x_dot_ref)^2
    }
    return J;
 }


// get lambda gradients, this is the constrained part. 
VectorXd Constraints(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s)
{	//// These are the gradient in respect of each of the constraints
    double dt0_low  =  0.2 - (current_time + dt_s(0)); //dt_min
    double dt0_high =  (current_time + dt_s(0)) - dt_max;
    double dt1_low  =  dt_min - dt_s(1);
    double dt1_high =  dt_s(1) - dt_max;
    double dt2_low  =  dt_min - dt_s(2);
    double dt2_high =  dt_s(2) - dt_max;

	////Gradients in respect to the lagrange multipliers  ///// Essentially the change in w:   dL/dw
    double l_grad1 = pow(u_s(0, 1) - x_s(0, 1), 2) + pow(u_s(1, 1) - x_s(1, 1), 2) - pow(0.5*sqrt(pow(L,2) - pow(h,2)),2);
    double l_grad2 = pow(u_s(0, 2) - x_s(0, 2), 2) + pow(u_s(1, 2) - x_s(1, 2), 2) - pow(0.5*sqrt(pow(L,2) - pow(h,2)),2);
    double l_grad3 = pow(u_s(0, 1) - x_s(0, 2), 2) + pow(u_s(1, 1) - x_s(1, 2), 2) - pow(0.5*sqrt(pow(L,2) - pow(h,2)),2);
    double l_grad4 = pow(u_s(0, 2) - x_s(0, 3), 2) + pow(u_s(1, 2) - x_s(1, 3), 2) - pow(0.5*sqrt(pow(L,2) - pow(h,2)),2);

    double foot_grad1 = rfoot - foot_flag * (u_s(1, 1) - u_s(1, 0));
    double foot_grad2 = rfoot + foot_flag * (u_s(1, 2) - u_s(1, 1));

    VectorXd inEquality_grad(12);
    inEquality_grad << 	dt0_low,	// 1
						dt0_high,	// 2
						dt1_low, 	// 3
						dt1_high, 	// 4
						dt2_low, 	// 5
						dt2_high, 	// 6
						l_grad1, 	// 7
						l_grad2, 	// 8
						l_grad3, 	// 9
						l_grad4, 	// 10
						foot_grad1, // 11
						foot_grad2;	// 12

//    cout << "inEquality_grad is " << inEquality_grad << endl;
//    inEquality_grad = inEquality_grad/inEquality_grad.norm();
    return inEquality_grad;
}


////These are the callbacks. Note that we don't use RK4 anymore
// Callback function for receiving the planner outputs
void footstepPlanCallbackRK4(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
//    cout<< "received rk4 data " << endl;
    u_s_rk4(0,1) = msg->data[0];
    u_s_rk4(1,1) = msg->data[1];
    u_s_rk4(0,2) = msg->data[2];
    u_s_rk4(1,2) = msg->data[3];
    t_s_rk4(0) = msg->data[4];
    t_s_rk4(1) = msg->data[5];
    t_s_rk4(2) = msg->data[6];
    u_s = u_s_rk4;
    dt_s = t_s_rk4;
    support_foot_flag = msg->data[7];
//    cout << "--------------------------------" << endl;
//    cout<< "received u \n" << u_s << endl;
//    cout<< "received t \n" << dt_s << endl;

    solution.x_k = VectorXd::Zero(7);
	solution.x_k.segment(0, 3) = dt_s.transpose();
	solution.x_k.segment(3, 2) = u_s.block(0, 1, 1, 2).transpose();
	solution.x_k.segment(5, 2) = u_s.block(1, 1, 1, 2).transpose();
	solution.delta_x_k = VectorXd::Zero(7);
	solution.y_k = VectorXd::Zero(7);
	solution.B_k = Matrix<double, 7, 7>::Identity();
	solution.H_k = Matrix<double, 7, 7>::Identity();
	rk4_received = true;
}



/////This receives a message from whole body controller. Then sets the values to the data
void plannerInputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    x_0(0) = msg->data[0];
    x_dot_0(0) = msg->data[1];
    x_dot_dot_0(0) = msg->data[2];
    x_0(1) = msg->data[3];
    x_dot_0(1) = msg->data[4];
    x_dot_dot_0(1) = msg->data[5];
    left_x  = msg->data[6];
    left_y  = msg->data[7];
    right_x = msg->data[8];
    right_y = msg->data[9];
    foot_flag = msg->data[10];
    current_time = msg->data[11];
//    cout << "received states " << endl;
//    cout << "x " << x_0(0) << endl;
//    cout << "x dot " << x_dot_0(0) << endl;
//    cout << x_dot_dot_0(0) << endl;
//    cout << "y " << x_0(1) << endl;
//    cout << "y dot " << x_dot_0(1) << endl;
//    cout << x_dot_dot_0(1) << endl;
//    cout << left_x << endl;
//    cout << left_y << endl;
//    cout << right_x << endl;
//    cout << right_y << endl;
//    cout << foot_flag << endl;
//    cout << current_time << endl;
}



////Setting the weights (i.e. Lagrange multipliers) from the most recent data
void weightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    w[0] = msg->data[0];	// lambda    dt0
    w[1] = msg->data[1];	// lambda    dt0
    w[2] = msg->data[2];	// lambda    dt1
    w[3] = msg->data[3];	// lambda    dt1
    w[4] = msg->data[4];	// lambda    dt2
    w[5] = msg->data[5];	// lambda    dt2
    w[6] = msg->data[6];	// lambda1   lmax
    w[7] = msg->data[7];	// lambda2   lmax
    w[8] = msg->data[8];	// lambda3   lmax
    w[9] = msg->data[9];	// lambda4   lmax
    w[10] = msg->data[10];	// lambda1   rfoot
    w[11] = msg->data[11];	// lambda2   rfoot
   cout << "The received weights are " << w << endl;
   w_received = true;
}

// Setting Jacobian in respect to weight. In case you are using second order approximation (which we dont)
void jacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	std::vector<double> data = msg->data;
	Eigen::Map<Eigen::MatrixXd> mat(data.data(), 30, 16);
	Jacobian = mat;
//	cout<<"The received jacobian is " << Jacobian << endl;
}

//Function that receives the Jacobian and converts to Hessian (NOTE: but no worries, we don't need that)
void hessianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	std::vector<double> data = msg->data;
	Eigen::Map<Eigen::MatrixXd> mat(data.data(), 19, 19);
}



ofstream myFile;

int main(int argc, char ** argv){
    Vector4d x_opt,y_opt;
    const double LOOP_RATE = 200;
    int iter_main = 0;
    ros::init(argc, argv, "gradient_descent_footPlanner_node");
    myFile.open("COM_AL.csv"); // save data

	//////Suscribers that need to be used (i.e. Planner input) for the test environment
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Publisher planner_output_pub = n.advertise<std_msgs::Float64MultiArray>("/time_slider_gazebo/footstep_plan", 1);
    ros::Publisher planner_gradient_pub = n.advertise<std_msgs::Float64MultiArray>("/time_slider_gazebo/gd_gradients", 1);
    ros::Subscriber sub_planner_input = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/planner_input", 'None', plannerInputCallback);
    ros::Subscriber sub_RK4_planner_input = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/footstep_plan_rk4", 'None', footstepPlanCallbackRK4);
    ros::Subscriber sub_weight        = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/cost_weight", 'None', weightCallback);
//    ros::Subscriber sub_jacobian      = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/jacobian", 'None', jacobianCallback);
//    ros::Subscriber sub_hessian       = n.subscribe<std_msgs::Float64MultiArray>("/time_slider_gazebo/hessianInv", 'None', hessianCallback);

    support_foot_flag = foot_flag;
	///// Getting the existing solutions 
	Jacobian = MatrixXd::Zero(30, 16);
	HessianInv = MatrixXd::Zero(19, 19);
	solution.x_k = VectorXd::Zero(7);
    w  = VectorXd::Zero(n_weights);
//    dyn_w  = VectorXd::Zero(n_weights);
    cout << "started" << endl

    MatrixXd x_s, xdot_s, xddot_s; /////We are defining x_s for x and y 
	x_s = MatrixXd::Zero(2, 4);
	xdot_s = MatrixXd::Zero(2, 4);	
	xddot_s = MatrixXd::Zero(2, 4);

	x_s(0, 0) = x_0(0);
	x_s(1, 0) = x_0(1);
	xdot_s(0, 0) = x_dot_0(0);
	xdot_s(1, 0) = x_dot_0(1);
	xddot_s(0, 0) = xddot(0.0, u_s(0, 0), x_s(0, 0), xdot_s(0, 0));
	xddot_s(1, 0) = xddot(0.0, u_s(1, 0), x_s(1, 0), xdot_s(1, 0));

	for (int k = 0; k < 3; k++) {
		x_s(0, k + 1) = x(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
		x_s(1, k + 1) = x(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
		xdot_s(0, k + 1) = xdot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
		xdot_s(1, k + 1) = xdot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
		xddot_s(0, k + 1) = xddot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
		xddot_s(1, k + 1) = xddot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
	}
	//cout << "u_s" << u_s << endl;
	//cout << "dT_s" << dt_s << endl;
	//cout << "x_s" << x_s << endl;
	//cout << "xdot_s" << xdot_s << endl;
	//cout << "xddot_s" << xddot_s << endl;

	// calculate gradients at step k
	Vector3d grad_t = dJ_dt(dt_s, u_s, x_s, xdot_s, xddot_s);
	Vector2d grad_ux = dJ_dux(dt_s, u_s, x_s, xdot_s, xddot_s);
	Vector2d grad_uy = dJ_duy(dt_s, u_s, x_s, xdot_s, xddot_s);
//	cout<< "calculate dJ_dt, first iteration" << endl;
//	cout<< "calculate dJ_dux, first iteration" << endl;
//	cout<< "calculate dJ_duy, first iteration" << endl;

	VectorXd grad_k(7);
	grad_k << grad_t, grad_ux, grad_uy;

	VectorXd lamda_grad;
	double lamda_step_size = 0.005;
	double step_size = 1e-4;

	//cout << "The weight is " << w << endl;
	//cout << "The gradients are " << grad_k << endl;
	MatrixXd J_dxdt(4,3);
	MatrixXd J_dydt(4,3);
	MatrixXd w_buffer(3,12);
	w_buffer << 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0;

//	J_dxdt = MatrixXd::Zero(4,3);

    while (ros::ok()){
        auto t1 = std::chrono::high_resolution_clock::now();
//            cout << "u ini is \t" << u_s << endl;
//            cout << "t ini is \t" << dt_s << endl;
//            cout << "current robot COM pos is \t" << x_0 << endl;
//            cout << "current robot COM vel is \t" << x_dot_0 << endl;

        ////////////// Gradient Descent ////////////////////////////////
        if (foot_flag == -1){
            u_s(0,0) = left_x;
            u_s(1,0) = left_y;
        }
        else if (foot_flag == 1){
            u_s(0,0) = right_x;
            u_s(1,0) = right_y;
        }

     	MatrixXd x_s, xdot_s, xddot_s;
		x_s = MatrixXd::Zero(2, 4);
		xdot_s = MatrixXd::Zero(2, 4);
		xddot_s = MatrixXd::Zero(2, 4);
		x_s(0, 0) = x_0(0);
		x_s(1, 0) = x_0(1);
		xdot_s(0, 0) = x_dot_0(0);
		xdot_s(1, 0) = x_dot_0(1);
		xddot_s(0, 0) = xddot(0.0, u_s(0, 0), x_s(0, 0), xdot_s(0, 0));
		xddot_s(1, 0) = xddot(0.0, u_s(1, 0), x_s(1, 0), xdot_s(1, 0));
		for (int k = 0; k < 3; k++) {
			x_s(0, k + 1) = x(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
			x_s(1, k + 1) = x(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
			xdot_s(0, k + 1) = xdot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
			xdot_s(1, k + 1) = xdot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
			xddot_s(0, k + 1) = xddot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
			xddot_s(1, k + 1) = xddot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
		}
		//cout << "x_s" << x_s << endl;
		//cout << "xdot_s" << xdot_s << endl;
		//cout << "u_s " << u_s << endl;


        // calculate gradients at step k
        Vector3d grad_t = dJ_dt(dt_s, u_s, x_s, xdot_s, xddot_s);
        Vector2d grad_ux = dJ_dux(dt_s, u_s, x_s, xdot_s, xddot_s);
        Vector2d grad_uy = dJ_duy(dt_s, u_s, x_s, xdot_s, xddot_s);

        VectorXd grad_k(7);  ////?????????????????????????????????????????????????????????????????????????????
        grad_k << grad_t, grad_ux, grad_uy;
//            cout << "The gradients are " << grad_k << endl;

		//// GRADIENT descent of footstep position u and lagrange multiplier lambda, from here down----   
        VectorXd grad_k_new = grad_k;
        MatrixXd x_new, xdot_new, xddot_new;
        x_new = x_s;
        xdot_new = xdot_s;
        xddot_new = xddot_s;

		double mu_step_size = 0.2;//
		double phi_scaling  = 1.0001;
		double countz = 0;
		double alphaz = 0.006;

		myFile << "\n";

        while(grad_k_new.norm()> 0.05 && countz < 35000){   		/// WHILE the solution is not satisfying (i.e. larger than an arbitrary 0.05) the tollerace, keep iterating until certain number of iterations
//          cout << "grad norm is " << grad_k_new.norm() << endl;
		    double cost1 = J(dt_s, u_s, x_new, xdot_new, xddot_new);
//          UPDATE stuff for u and dt:
            dt_s -= step_size*grad_k_new.segment(0,3)*1.0;				///dt is Length of the footstep. Here we are doing a simple gradient descent
            u_s.block(0, 1, 1, 2) -= step_size*grad_k_new.segment(3,2).transpose();// /2;	///u is Position of footstep x
            u_s.block(1, 1, 1, 2) -= step_size*grad_k_new.segment(5,2).transpose();	///u is Position of footstep y        ---> both dt and u above are updated
//          cout<< "result dt is " << dt_s << endl;									      ---> NOTE: No 'z'
//          cout << "u_s is " << u_s << endl;
		    lamda_grad = Constraints(dt_s, u_s, x_new, xdot_new, xddot_new);  	//// Here we get the gradient of the cost function in respect of the Lagrange multipliers (i.e. dJ/dLamda)

			w.segment(0,1) += mu_step_size * lamda_grad.segment(0,1)*6;
			w.segment(1,1) += mu_step_size * lamda_grad.segment(1,1)*6;
		    w.segment(2,8) += mu_step_size * lamda_grad.segment(2,8)*6;  //// This is UPDATING the lagrangian multipliers USING MU----------------
		    w.segment(10,2) += mu_step_size * lamda_grad.segment(10,2)*6;  //// This is UPDATING the lagrangian multipliers USING MU----------------

//////////////////////////////////////For slopes

            w_received = false;
            // cout<< 'buffer is:::::::::: ' << w_buffer << endl;


            for(int i=0; i<n_weights; i++){  		      //// Also Lagrange Multipliers need to be >=0, otherwise they are put =0
                if(w[i]<0) w[i] = 0;
                // do a smoothing if w too large   
                if(w[i]>10000000){
                	w[i] = (w_buffer(0,i)+w_buffer(1,i)+w_buffer(2,i))/3;
                }
                
                w_buffer(0,i) = w_buffer(1,i);
                w_buffer(1,i) = w_buffer(2,i);
                w_buffer(2,i) = w[i];

            }
		
		    mu_step_size = phi_scaling * mu_step_size;
		    //cout << "the mu_step_size is: " << mu_step_size;
            // project to positive side
		    //// From here we are setting up the dynamic model that the robot will do after it is optimized fom the above steps
            x_new = MatrixXd::Zero(2, 4);
            x_new(0, 0) = x_0(0);
            x_new(1, 0) = x_0(1);
            xdot_new = MatrixXd::Zero(2, 4);
            xdot_new(0, 0) = x_dot_0(0);
            xdot_new(1, 0) = x_dot_0(1);
            xddot_new = MatrixXd::Zero(2, 4);
            xddot_new(0, 0) = xddot(0.0, u_s(0, 0), x_new(0, 0), xdot_new(0, 0));
            xddot_new(1, 0) = xddot(0.0, u_s(1, 0), x_new(1, 0), xdot_new(1, 0));

		    countz += 1;
		    double cost2 = J(dt_s, u_s, x_new, xdot_new, xddot_new);		// calculate cost
		    //cout << "COST is: " << (cost1-cost2);
		    //cout << " in number of ite:  " << countz << endl;
		    myFile << countz << ",";

            for (int k = 0; k < 3; k++){  ////3 FOOTSTEPS in the future ----- ARTO does the optimization over the next 3 footsteps------------------------------------------------------------------------------------------------------------
                x_new(0, k + 1) = x(dt_s[k], u_s(0, k), x_new(0, k), xdot_new(0, k));
                x_new(1, k + 1) = x(dt_s[k], u_s(1, k), x_new(1, k), xdot_new(1, k));
                xdot_new(0, k + 1) = xdot(dt_s[k], u_s(0, k), x_new(0, k), xdot_new(0, k));
                xdot_new(1, k + 1) = xdot(dt_s[k], u_s(1, k), x_new(1, k), xdot_new(1, k));
                xddot_new(0, k + 1) = xddot(dt_s[k], u_s(0, k), x_new(0, k), xdot_new(0, k));
                xddot_new(1, k + 1) = xddot(dt_s[k], u_s(1, k), x_new(1, k), xdot_new(1, k));
            }
            Vector3d grad_t_new = dJ_dt(dt_s, u_s, x_new, xdot_new, xddot_new);
            Vector2d grad_ux_new = dJ_dux(dt_s, u_s, x_new, xdot_new, xddot_new);
            Vector2d grad_uy_new = dJ_duy(dt_s, u_s, x_new, xdot_new, xddot_new);
            grad_k_new << grad_t_new, grad_ux_new, grad_uy_new;
			//cout << "grad is " << grad_k_new << endl;
        }
		//cout << "lambda grad is " << lamda_grad << endl;
        myFile << "\n";

    if (rk4_received){

        	if(dt_s(0)>=0){
        		dt_s_backup(0) = dt_s(0);
        	}
        	else if(dt_s_backup(0)-1/LOOP_RATE < 0){
        		dt_s(0) = 0.00001;
        	}
        	else{
        		dt_s(0) = dt_s_backup(0)-1/LOOP_RATE;
        	}
			// project t inside constraint
        	for(int j=0; j<3; j++){
        		if(j>0){
                	if(dt_s(j)<0.4){
                		dt_s(j) = 0.4;
                		cout << "PERFORMED: Projection on 0.4 sideeeeeeeeeeeeeeeeeeee" << endl;
                	} 
                }

                if(j == 0 && dt_s(j)+current_time > 0.6){dt_s(j) = max(0.4-current_time, 0.05);}

            	if(j>0 && dt_s(j)>0.6){
            		if(j == 0){dt_s(j) = 0.6-current_time;}
            		dt_s(j) = 0.6;
            		cout << "PERFORMED: Projection on 0.6 sideeeeeeeeeeeeeeeeeeee" << endl;
            	} 
        	}

			// project u
        	if (abs(u_s(0,0)-u_s(0,1))>sqrt(lmax)){         //lmax				x 0-1
        		u_s(0,1) = u_s(0,0)+sqrt(lmax)*0.8;
        	}
        	if (abs(u_s(0,1)-u_s(0,2))>sqrt(lmax)){         //lmax				x 1-2
        		u_s(0,2) = u_s(0,1)+sqrt(lmax)*0.8;
        	}
        	if (abs(u_s(1,0)-u_s(1,1))>sqrt(lmax)){         //lmax				y 0-1
        		u_s(1,1) = u_s(1,0)+foot_flag*sqrt(lmax)*0.3;
        	}
        	if (abs(u_s(1,1)-u_s(1,2))>sqrt(lmax)){         //lmax				y 1-2
        		u_s(1,2) = u_s(1,1)+foot_flag*sqrt(lmax)*0.3;
        	}

        	//  Constraints for the crossing in y
        	if (abs(u_s(1,0)-u_s(1,1))<rfoot && abs(u_s(0,0)-u_s(0,1))<rfoot*1){
        		u_s(1,1) = u_s(1,0)+foot_flag*rfoot;         									//rfoot			y 0-1
        	}
        	if (abs(u_s(1,1)-u_s(1,2))<rfoot && abs(u_s(0,1)-u_s(0,2))<rfoot*1){              //rfoot				y 1-2
        		u_s(1,2) = u_s(1,1)+foot_flag*rfoot;
        	}
        }

        cout << "The final weights is " << w << endl;
		

                //}//for						/////////NOTE: you need to inser the solutions above in a for/while loop to converge them. Currently they are done just once.

        cout << "--------------- Final Solution ---------------------" << endl;
        cout<< "result dt is " << dt_s << endl;
        cout << "result u_s is " << u_s << endl;
        cout << "----------------------------------------------------" << endl;
        x_s = MatrixXd::Zero(2, 4);
        x_s(0, 0) = x_0(0);
        x_s(1, 0) = x_0(1);
        xdot_s = MatrixXd::Zero(2, 4);
        xdot_s(0, 0) = x_dot_0(0);
        xdot_s(1, 0) = x_dot_0(1);
        xddot_s = MatrixXd::Zero(2, 4);
        xddot_s(0, 0) = xddot(0.0, u_s(0, 0), x_s(0, 0), xdot_s(0, 0));
        xddot_s(1, 0) = xddot(0.0, u_s(1, 0), x_s(1, 0), xdot_s(1, 0));

        for (int k = 0; k < 3; k++) {
            x_s(0, k + 1) = x(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
            x_s(1, k + 1) = x(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
            xdot_s(0, k + 1) = xdot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
            xdot_s(1, k + 1) = xdot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
            xddot_s(0, k + 1) = xddot(dt_s[k], u_s(0, k), x_s(0, k), xdot_s(0, k));
            xddot_s(1, k + 1) = xddot(dt_s[k], u_s(1, k), x_s(1, k), xdot_s(1, k));
        }

//                if(dt_s(0)>2.5){
//                    dt_s = t_s_rk4;
//                    u_s = u_s_rk4;
//                }
//                cout << "Final State is " << endl;
//                cout << "X is \n" << x_s << endl;
//                cout << "X dot is \n" << xdot_s << endl;
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
            //cout << "Duration: \n" << duration << " microseconds" << endl;
//            }
//            rk4_received = false;

        if(support_foot_flag == foot_flag && dt_s(0) > 10/LOOP_RATE && !dt_s.hasNaN() && !u_s.hasNaN()  && dt_s(0)<0.6){
//            cout << "Publish Plan !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            std_msgs::Float64MultiArray footstep_plan;
            footstep_plan.data.push_back(u_s(0,1));
            footstep_plan.data.push_back(u_s(1,1));
            footstep_plan.data.push_back(u_s(0,2));
            footstep_plan.data.push_back(u_s(1,2));
            footstep_plan.data.push_back(dt_s(0));
            footstep_plan.data.push_back(dt_s(1));
            footstep_plan.data.push_back(support_foot_flag);
            planner_output_pub.publish(footstep_plan);
            if(flag==0){
                dt_s(0) -= 1/LOOP_RATE;
                dt_s(1) -= 1/LOOP_RATE;
                dt_s(2) -= 1/LOOP_RATE;
            }
            else if (flag==1){flag = 0;}
        } 
        else if (dt_s.hasNaN() || u_s.hasNaN()){
            cout << "Publishing RK4 -------------------" << endl;
            dt_s = t_s_rk4;
            u_s = u_s_rk4;
            std_msgs::Float64MultiArray footstep_plan;
            footstep_plan.data.push_back(u_s(0,1));
            footstep_plan.data.push_back(u_s(1,1));
            footstep_plan.data.push_back(u_s(0,2));
            footstep_plan.data.push_back(u_s(1,2));
            footstep_plan.data.push_back(dt_s(0));
            footstep_plan.data.push_back(dt_s(1));
            footstep_plan.data.push_back(support_foot_flag);
            planner_output_pub.publish(footstep_plan);
        }
        else{
            cout << "Not publishing -------------------" << endl;
            if(flag==0){
                dt_s(0) -= 1/LOOP_RATE;
                dt_s(1) -= 1/LOOP_RATE;
                dt_s(2) -= 1/LOOP_RATE;
            }
        }
        // to be modified
        if(support_foot_flag != foot_flag){
            // dt_s(0) = dt_s(1);
            // dt_s(1) = dt_s(2);
            // dt_s(2) = dt_s(2) + 0.4;//opt.time(3) - opt.time(2);
            // u_s(0,0) = u_s(0,1);
            // u_s(1,0) = u_s(1,1);
            // u_s(0,1) = u_s(0,2);
            // u_s(1,1) = u_s(1,2);
            // u_s(0,2) = u_s(0,2) + (u_s(0,2) - u_s(0,1));
            // u_s(1,2) = u_s(1,2) + (u_s(1,1) - u_s(1,2));
            // current_time = 0.0;
            // support_foot_flag = foot_flag;

            current_time = 0.0;
            support_foot_flag = foot_flag;
            dt_s(0) = 0.4;
            dt_s(1) = 0.4;
            dt_s(2) = 0.4;//opt.time(3) - opt.time(2);
            u_s(0,0) = u_s(0,0) + x_dot_ref(0)*0.4;
            u_s(1,0) = u_s(1,0) - foot_flag*rfoot;
            u_s(0,1) = u_s(0,0) + x_dot_ref(0)*0.4;
            u_s(1,1) = u_s(1,0) - foot_flag*rfoot*(-1);
            u_s(0,2) = u_s(0,1) + x_dot_ref(0)*0.4;
            u_s(1,2) = u_s(1,1) - foot_flag*rfoot;

            std_msgs::Float64MultiArray footstep_plan;
            footstep_plan.data.push_back(u_s(0,1));
            footstep_plan.data.push_back(u_s(1,1));
            footstep_plan.data.push_back(u_s(0,2));
            footstep_plan.data.push_back(u_s(1,2));
            footstep_plan.data.push_back(dt_s(0));
            footstep_plan.data.push_back(dt_s(1));
            footstep_plan.data.push_back(support_foot_flag);
            planner_output_pub.publish(footstep_plan);

            cout << "switching !!!!!!!!!!!!!!!!!!!!!!!!!!1!!!!!!!!!!!!" << endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }//while ok
    return 0;
}