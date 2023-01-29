#include "analytical_gradients.h"

using namespace Eigen;
using namespace std;
// Calculate state based on inverted pendulum equation
double x(double dt, double u, double x_prev, double x_dot_prev){
	double x_ = u + (x_prev - u) * cosh(omega * dt) + (x_dot_prev / omega) * sinh(omega * dt);
    return x_;
}

double xdot(double dt, double u, double x_prev, double x_dot_prev){
	double xdot_ = omega * (x_prev - u) * sinh(omega * dt) + x_dot_prev * cosh(omega * dt);
    return xdot_;
}

double xddot(double dt, double u, double x_prev, double x_dot_prev){
	double xddot_ = omega * omega * (x_prev - u) * cosh(omega * dt) + omega * x_dot_prev * sinh(omega * dt);
    return xddot_;
}

// Analytical gradients
double dxdot_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int kt=0){
	double dxdot_dt_ = 0.0;
	if (kx == kt+1){
		dxdot_dt_ = xddot_s[kx]; 
	}else if (kx > kt+1){
		dxdot_dt_ = dx_dt(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, kt) * omega * sinh(omega * dt_s[kx-1]) + dxdot_dt(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, kt) * cosh(omega * dt_s[kx-1]);
	}
	return dxdot_dt_;
}

double dx_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int kt=0){
    double dx_dt_ = 0.0;
	if (kx == kt+1){
		dx_dt_ = xdot_s[kx];
	}else if (kx > kt+1){
		dx_dt_ = dx_dt(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, kt) * cosh(omega * dt_s[kx-1]) + dxdot_dt(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, kt) * sinh(omega *dt_s[kx-1]) / omega;
	}
	return dx_dt_;
}
double dx_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int ku=1){
    double dx_du_ = 0.0;
	if (kx == ku + 1){
		dx_du_ =1 - cosh(omega * dt_s[kx - 1]);
	}else if (kx > ku + 1){
		dx_du_ = dx_du(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, ku) * cosh(omega * dt_s[kx-1]) + dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, ku) * sinh(omega * dt_s[kx-1]) / omega;
	}
	return dx_du_;
}

double dxdot_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int ku=1){
    double dxdot_du_ = 0.0;
	if (kx == ku + 1){
		dxdot_du_ = -omega * sinh(omega * dt_s[kx - 1]);
	}else if (kx > ku + 1){
		dxdot_du_ = dx_du(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, ku) * omega * sinh(omega * dt_s[kx-1]) + dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, kx-1, ku) * cosh(omega * dt_s[kx-1]);
	}
	return dxdot_du_;
}

double dJ_dux1(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s){
    double grad = 0;
    for(int j=2; j<4; j++){
        grad += 2 * dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, j, 1) * (xdot_s[j] - x_dot_ref[0])*Qx;}
    return grad;
}

double dJ_dux2(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s){
    double grad = 0;
    for(int j=3; j<4; j++){
        grad += 2 * dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, j, 2) * (xdot_s[j] - x_dot_ref[0])*Qx;}
    return grad;
}

double dJ_duy1(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s){
    double grad = 0;
    for(int j=2; j<4; j++){
        grad += 2 * dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, j, 1) * (xdot_s[j] - x_dot_ref[1])*Qy;}
    return grad;
}

double dJ_duy2(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s){
    double grad = 0;
    for(int j=3; j<4; j++){
        grad += 2 * dxdot_du(dt_s, u_s, x_s, xdot_s, xddot_s, j, 2) * (xdot_s[j] - x_dot_ref[1])*Qy;}
    return grad;
}

double dJ_dt0(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
    double grad = 0.0;
	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = xdot_s.row(0);
	Vector4d xdot_y = xdot_s.row(1);
	Vector4d xddot_x = xddot_s.row(0);
	Vector4d xddot_y = xddot_s.row(1);

    for(int j=1; j<4; j++){
        grad += 2 * dxdot_dt(dt_s, u_x, x_x, xdot_x, xddot_x, j, 0) * (xdot_x[j] - x_dot_ref[0])*Qx;
        grad += 2 * dxdot_dt(dt_s, u_y, x_y, xdot_y, xddot_y, j, 0) * (xdot_y[j] - x_dot_ref[1])*Qy;
    }
    return grad;
}

double dJ_dt1(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
    double grad = 0.0;
	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = xdot_s.row(0);
	Vector4d xdot_y = xdot_s.row(1);
	Vector4d xddot_x = xddot_s.row(0);
	Vector4d xddot_y = xddot_s.row(1);

    for(int j=1; j<4; j++){
        grad += 2 * dxdot_dt(dt_s, u_x, x_x, xdot_x, xddot_x, j, 1) * (xdot_x[j] - x_dot_ref[0])*Qx;
        grad += 2 * dxdot_dt(dt_s, u_y, x_y, xdot_y, xddot_y, j, 1) * (xdot_y[j] - x_dot_ref[1])*Qy;
    }
    return grad;
}

double dJ_dt2(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
    double grad = 0.0;
	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = xdot_s.row(0);
	Vector4d xdot_y = xdot_s.row(1);
	Vector4d xddot_x = xddot_s.row(0);
	Vector4d xddot_y = xddot_s.row(1);

    for(int j=1; j<4; j++){
        grad += 2 * dxdot_dt(dt_s, u_x, x_x, xdot_x, xddot_x, j, 2) * (xdot_x[j] - x_dot_ref[0])*Qx;   ///Derivative of the body velocity in respect to footstep duration, multiplied by the actual error
        grad += 2 * dxdot_dt(dt_s, u_y, x_y, xdot_y, xddot_y, j, 2) * (xdot_y[j] - x_dot_ref[1])*Qy;   ///and we sum both x and y together because its the sum of the squared error --> ALL this is based on the cost function
    }
    return grad;
}

Vector2d dJ_dux(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = xdot_s.row(0);
	Vector4d xdot_y = xdot_s.row(1);
	Vector4d xddot_x = xddot_s.row(0);
	Vector4d xddot_y = xddot_s.row(1);
	// x1u1,x2u2 stop the body from travelling too far away from the foot (based on how long the leg can be)
	double px1u1 = w[6] * ((x_s.col(1) - u_s.col(1)).transpose() * (x_s.col(1) - u_s.col(1)) - lmax);   
	double px2u2 = w[7] * ((x_s.col(2) - u_s.col(2)).transpose() * (x_s.col(2) - u_s.col(2)) - lmax);

	// And here we do the gradients of the 2 things above
	double dpx1u1_dux1 = -2 * (x_s(0, 1) - u_s(0, 1)) * w[6];
	double dpx2u2_dux1 = 2 * (dx_du(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 1)) * (x_s(0, 2) - u_s(0, 2)) * w[7];
	double dpx2u2_dux2 = -2 * (x_s(0, 2) - u_s(0, 2)) * w[7];

	double px2u1 = w[8] * ((x_s.col(2) - u_s.col(1)).transpse() * (x_s.col(2) - u_s.col(1)) - lmax);  /// x2u1, x3u2 stop the next foot to be places too far from the body
	double px3u2 = w[9] * ((x_s.col(3) - u_s.col(2)).transpose() * (x_s.col(3) - u_s.col(2)) - lmax);  /// x3u3: make sure that the CoM at the 3rd footstep is not too far from the 2nd step (1,2,3 are which footstep you are on)

	// And here we do the gradients of the 2 things above
	double dpx2u1_dux1 = 2 * (dx_du(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 1) - 1) * (x_s(0, 2) - u_s(0, 1)) * w[8];
	double dpx3u2_dux2 = 2 * (dx_du(dt_s, u_x, x_x, xdot_x, xddot_x, 3, 2) - 1) * (x_s(0, 3) - u_s(0, 2)) * w[9];
	double dpx3u2_dux1 = 2 * (dx_du(dt_s, u_x, x_x, xdot_x, xddot_x, 3, 1)) * (x_s(0, 3) - u_s(0, 2)) * w[9];

	// These penalties are added to the cost function. These are the multipliers penalties. You need to take the gradient of these in respect of each of the optimization variables (from which you get e.g. dpx1u1_dux1)
	double dux1_inequality_penalties = dpx1u1_dux1 + dpx2u2_dux1 + dpx2u1_dux1 + dpx3u2_dux1;
	double dux2_inequality_penalties = dpx2u2_dux2 + dpx3u2_dux2;
	
	double AL_dpx1u1_dux1 = 0.0;	// a
	double AL_dpx2u2_dux1 = 0.0;	// a
	double AL_dpx2u2_dux2 = 0.0;	// b	
	double AL_dpx2u1_dux1 = 0.0;	// a
	double AL_dpx3u2_dux2 = 0.0;	// b
	double AL_dpx3u2_dux1 = 0.0;	// a

	if ((px1u1/w[6]) <= 0){
		AL_dpx1u1_dux1 = 0;
	}else if ((px1u1/w[6]) > 0){
		AL_dpx1u1_dux1 = 2 * mu_step_size * (px1u1/w[6]) * dpx1u1_dux1/w[6];	// a
	}
	
	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_dux1 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_dux1 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_dux1/w[7];	// a
	}

	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_dux2 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_dux2 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_dux2/w[7];	// b	
	}

	if ((px2u1/w[8]) <= 0){
		AL_dpx2u1_dux1 = 0;
	}else if ((px2u1/w[8]) > 0){
		AL_dpx2u1_dux1 = 2 * mu_step_size * (px2u1/w[8]) * dpx2u1_dux1/w[8];	// a
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_dux2 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_dux2 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_dux2/w[9];	// b
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_dux1 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_dux1 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_dux1/w[9];	// a
	}

	double AL_dux1_inequality_penalties = AL_dpx1u1_dux1 + AL_dpx2u2_dux1 + AL_dpx2u1_dux1 + AL_dpx3u2_dux1;
	double AL_dux2_inequality_penalties = AL_dpx2u2_dux2 + AL_dpx3u2_dux2;

	//  Debug
	//	cout << "dx inequality penalty " << endl;
	//	cout << dpx1u1_dux1 <<  endl;
	//	cout << dpx2u2_dux1 <<  endl;
	//	cout << dpx2u1_dux1 <<  endl;
	//	cout << dpx3u2_dux1 <<  endl;
	//	cout << dpx2u2_dux2 <<  endl;
	//	cout << dpx3u2_dux2 <<  endl;

	//    cout << "dJ with dux gradient" << endl;
	//    cout << dJ_dux1(dt_s, u_x, x_x, xdot_x, xddot_x) << endl;
	//    cout << dJ_dux2(dt_s, u_x, x_x, xdot_x, xddot_x) << endl;

    Vector2d dJ_dux_;
    dJ_dux_ << dJ_dux1(dt_s, u_x, x_x, xdot_x, xddot_x) + dux1_inequality_penalties + AL_dux1_inequality_penalties, //dJ_dux1(dt_s, u_x, x_x, xdot_x, xddot_x) -
               dJ_dux2(dt_s, u_x, x_x, xdot_x, xddot_x) + dux2_inequality_penalties + AL_dux2_inequality_penalties; // dJ_dux2(dt_s, u_x, x_x, xdot_x, xddot_x) -
    return dJ_dux_;
}
 
Vector2d dJ_duy(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> x_dot_s, Matrix<double,2, 4> xd_dot_s){
	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = x_dot_s.row(0);
	Vector4d xdot_y = x_dot_s.row(1);
	Vector4d xddot_x = xd_dot_s.row(0);
	Vector4d xddot_y = xd_dot_s.row(1);
	
	double px1u1 = w[6] * ((x_s.col(1) - u_s.col(1)).transpose() * (x_s.col(1) - u_s.col(1)) - lmax);
	double px2u2 = w[7] * ((x_s.col(2) - u_s.col(2)).transpose() * (x_s.col(2) - u_s.col(2)) - lmax);

	double dpx1u1_duy1 = -2 * (x_s(1, 1) - u_s(1, 1)) * w[6];
	double dpx2u2_duy1 = 2 * (dx_du(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 1)) * (x_s(1, 2) - u_s(1, 2)) * w[7];
	double dpx2u2_duy2 = -2 * (x_s(1, 2) - u_s(1, 2)) * w[7];

	double px2u1 = w[8] * ((x_s.col(2) - u_s.col(1)).transpose() * (x_s.col(2) - u_s.col(1)) - lmax);
	double px3u2 = w[9] * ((x_s.col(3) - u_s.col(2)).transpose() * (x_s.col(3) - u_s.col(2)) - lmax);
    
	double dpx2u1_duy1 = 2 * (dx_du(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 1) - 1) * (x_s(1, 2) - u_s(1, 1)) * w[8];
	double dpx3u2_duy2 = 2 * (dx_du(dt_s, u_y, x_y, xdot_y, xddot_y, 3, 2) - 1) * (x_s(1, 3) - u_s(1, 2)) * w[9];
	double dpx3u2_duy1 = 2 * (dx_du(dt_s, u_y, x_y, xdot_y, xddot_y, 3, 1)) * (x_s(1, 3) - u_s(1, 2)) * w[9];


//    cout<< "inEquality constraint in y" << endl;
//    cout << dpx1u1_duy1 << endl;
//    cout << dpx2u2_duy2 << endl;
//    cout << dpx2u1_duy1 << endl;
//    cout << dpx3u2_duy2 << endl;

	///foot_flag is which leg you are stepping with
	///The foot_flag doesn't matter with x, but it matters with y.
	double pu1u0 = w[10] * (foot_flag * (u_s(1, 1) - u_s(1, 0)) - rfoot); ///This makes sure the the 2 feet don't step on the top of each other
	double pu2u1 = w[11] * (-foot_flag * (u_s(1, 2) - u_s(1, 1)) - rfoot); ///rfoot is the radius of the foot and the flag will tell you how you need to keep the foot (left or right?)
								
	double dpu1u0_duy1 = -foot_flag * w[10];
	double dpu2u1_duy1 = -foot_flag * w[11];
	double dpu2u1_duy2 = foot_flag * w[11];

	double duy1_inequality_penalties = dpx1u1_duy1 + dpx2u2_duy1 + dpx2u1_duy1 + dpx3u2_duy1 + dpu1u0_duy1 + dpu2u1_duy1;
	double duy2_inequality_penalties = dpx2u2_duy2 + dpx3u2_duy2 + dpu2u1_duy2;	

	double AL_dpx1u1_duy1 = 0.0;	// a
	double AL_dpx2u2_duy1 = 0.0;	// a
	double AL_dpx2u2_duy2 = 0.0;	// b	
	double AL_dpx2u1_duy1 = 0.0;	// a
	double AL_dpx3u2_duy2 = 0.0;	// b
	double AL_dpx3u2_duy1 = 0.0;	// a

	double AL_dpu1u0_duy1 = 0.0;	// a
	double AL_dpu2u1_duy1 = 0.0;	// a
	double AL_dpu2u1_duy2 = 0.0;	// b

	if ((px1u1/w[6]) <= 0){
		AL_dpx1u1_duy1 = 0;
	}else if ((px1u1/w[6]) > 0){
		AL_dpx1u1_duy1 = 2 * mu_step_size * (px1u1/w[6]) * dpx1u1_duy1/w[6];	// a
	}

	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_duy1 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_duy1 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_duy1/w[7];	// a
	}

	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_duy2 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_duy2 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_duy2/w[7];	// b
	}

	if ((px2u1/w[8]) <= 0){
		AL_dpx2u1_duy1 = 0;
	}else if ((px2u1/w[8]) > 0){
		AL_dpx2u1_duy1 = 2 * mu_step_size * (px2u1/w[8]) * dpx2u1_duy1/w[8];	// a
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_duy2 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_duy2 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_duy2/w[9];	// b
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_duy1 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_duy1 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_duy1/w[9];	// a
	}

	if ((pu1u0/w[10]) <= 0){
		AL_dpu1u0_duy1 = 0;
	}else if ((pu1u0/w[10]) > 0){
		AL_dpu1u0_duy1 = 2 * mu_step_size * (pu1u0/w[10]) * dpu1u0_duy1/w[10];	// a
	}

	if ((pu2u1/w[11]) <= 0){
		AL_dpu2u1_duy1 = 0;
	}else if ((pu2u1/w[11]) > 0){
		AL_dpu2u1_duy1 = 2 * mu_step_size * (pu2u1/w[11]) * dpu2u1_duy1/w[11];	// a
	}

	if ((pu2u1/w[11]) <= 0){
		AL_dpu2u1_duy2 = 0;
	}else if ((pu2u1/w[11]) > 0){
		AL_dpu2u1_duy2 = 2 * mu_step_size * (pu2u1/w[11]) * dpu2u1_duy2/w[11];	// b
	}

	double AL_duy1_inequality_penalties = AL_dpx1u1_duy1 + AL_dpx2u2_duy1 + AL_dpx2u1_duy1 + AL_dpx3u2_duy1 + AL_dpu1u0_duy1 + AL_dpu2u1_duy1;
	double AL_duy2_inequality_penalties = AL_dpx2u2_duy2 + AL_dpx3u2_duy2 + AL_dpu2u1_duy2;

//    cout << "dy inequality penalty " << endl;
//    cout << dpx1u1_duy1 << endl;
//    cout << dpx2u2_duy1 << endl;
//    cout << dpx2u1_duy1 << endl;
//    cout << dpx3u2_duy1 << endl;
//    cout << dpu1u0_duy1 << endl;
//    cout << dpu2u1_duy1 << endl;
//    cout << dpx2u2_duy2 << endl;
//    cout << dpx3u2_duy2 << endl;
//    cout << dpu2u1_duy2 << endl;
//    cout << "dJ with duy gradient" << endl;
//    cout << dJ_duy1(dt_s, u_y, x_y, xdot_y, xddot_y) << endl;
//    cout << dJ_duy2(dt_s, u_y, x_y, xdot_y, xddot_y) << endl;

    Vector2d dJ_duy_;
    dJ_duy_ << dJ_duy1(dt_s, u_y, x_y, xdot_y, xddot_y) + duy1_inequality_penalties + AL_duy1_inequality_penalties,
               dJ_duy2(dt_s, u_y, x_y, xdot_y, xddot_y) + duy2_inequality_penalties + AL_duy2_inequality_penalties;
    return dJ_duy_;
}

Vector3d dJ_dt(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s){
    double dt0 = dt_s[0];
    double dt0_low = max(dt0_min - current_time, 0.001);
    double dt0_high = dt_max;
    double dt1 = dt_s[1];
    double dt1_low = dt_min;
    double dt1_high = dt_max;
    double dt2 = dt_s[2];
    double dt2_low = dt_min;
    double dt2_high = dt_max;

	Vector3d u_x = u_s.row(0);
	Vector3d u_y = u_s.row(1);
	Vector4d x_x = x_s.row(0);
	Vector4d x_y = x_s.row(1);
	Vector4d xdot_x = xdot_s.row(0);
	Vector4d xdot_y = xdot_s.row(1);
	Vector4d xddot_x = xddot_s.row(0);
	Vector4d xddot_y = xddot_s.row(1);

	// first 6 are making sure that minimum and maximum time are within constraints
    double dpt_dt0 = -w[0] + w[1];
    double dpt_dt1 = -w[2] + w[3]; 
    double dpt_dt2 = -w[4] + w[5];

	// make sure the next step isn't too far away
    double px1u1 = w[6] * ((x_s.col(1) - u_s.col(1)).transpose() * (x_s.col(1) - u_s.col(1)) - lmax);
    double px2u2 = w[7] * ((x_s.col(2) - u_s.col(2)).transpose() * (x_s.col(2) - u_s.col(2)) - lmax);

    VectorXd v1(2); v1 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 1, 0) ,
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 1, 0) ;
    double dpx1u1_dt0 = 2 * w[6] * v1.transpose() * (x_s.col(1) - u_s.col(1));

    VectorXd v2(2); v2 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 0),
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 0);
    double dpx2u2_dt0 = 2 * w[7] * v2.transpose() * (x_s.col(2) - u_s.col(2));

    VectorXd v3(2); v3 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 1),
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 1);
    double dpx2u2_dt1 = 2 * w[7] * v3.transpose() * (x_s.col(2) - u_s.col(2));


	// make sure the current step isn't too far
    double px2u1 = w[8] * ((x_s.col(2) - u_s.col(1)).transpose() * (x_s.col(2) - u_s.col(1)) - lmax);
    double px3u2 = w[9] * ((x_s.col(3) - u_s.col(2)).transpose() * (x_s.col(3) - u_s.col(2)) - lmax);

    VectorXd v4(2); v4 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 0),
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 0);
    double dpx2u1_dt0 = 2 * w[8] * v4.transpose() * (x_s.col(2) - u_s.col(1));

    VectorXd v5(2); v5 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 3, 0),
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 3, 0);
    double dpx3u2_dt0 = 2 * w[9] * v5.transpose() * (x_s.col(3) - u_s.col(2));

    VectorXd v6(2); v6 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 2, 1),
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 2, 1);
    double dpx2u1_dt1 = 2 * w[8] * v6.transpose() * (x_s.col(2) - u_s.col(1));

    VectorXd v7(2); v7 <<	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 3, 1);
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 3, 1);
    double dpx3u2_dt1 = 2 * w[9] * v7.transpose() * (x_s.col(3) - u_s.col(2));

    VectorXd v8(2); v8 << 	dx_dt(dt_s, u_x, x_x, xdot_x, xddot_x, 3, 2), 
							dx_dt(dt_s, u_y, x_y, xdot_y, xddot_y, 3, 2);
    double dpx3u2_dt2 = 2 * w[9] * v8.transpose() * (x_s.col(3) - u_s.col(2));

    double dt0_inequality_penalties = dpt_dt0 + dpx1u1_dt0 + dpx2u2_dt0 + dpx2u1_dt0 + dpx3u2_dt0;
    double dt1_inequality_penalties = dpt_dt1 + dpx2u2_dt1 + dpx2u1_dt1 + dpx3u2_dt1;
    double dt2_inequality_penalties = dpt_dt2 + dpx3u2_dt2;

	double AL_dpt_dt0_low = 0.0;
	double AL_dpt_dt1_low = 0.0;
	double AL_dpt_dt2_low = 0.0;

	double AL_dpt_dt0_high = 0.0;
	double AL_dpt_dt1_high = 0.0;
	double AL_dpt_dt2_high = 0.0;

	double AL_dpx1u1_dt0 = 0.0; 
	double AL_dpx2u2_dt0 = 0.0;
	double AL_dpx2u2_dt1 = 0.0;

	double AL_dpx2u1_dt0 = 0.0;
	double AL_dpx3u2_dt0 = 0.0;
	double AL_dpx2u1_dt1 = 0.0;
	double AL_dpx3u2_dt1 = 0.0;
	double AL_dpx3u2_dt2 = 0.0;


	if ((-dt0 + dt0_low) <= 0){
		AL_dpt_dt0_low = 0;
	}else if ((-dt0 + dt0_low) > 0){
		AL_dpt_dt0_low = 2 * mu_step_size * (-1 * (-dt0 + dt0_low));
	}
	if ((dt0 - dt0_high) <= 0){
		AL_dpt_dt0_high = 0;
	}else if ((dt0 - dt0_high) > 0){
		AL_dpt_dt0_high = 2 * mu_step_size * (1 * (dt0 - dt0_high));
	}

	if ((-dt1 + dt1_low) <= 0){
		AL_dpt_dt1_low = 0;
	}else if ((-dt1 + dt1_low) > 0){
		AL_dpt_dt1_low = 2 * mu_step_size * (-1 * (-dt1 + dt1_low));
	}
	if ((dt1 - dt1_high) <= 0){
		AL_dpt_dt1_high = 0;
	}else if ((dt1 - dt1_high) > 0){
		AL_dpt_dt1_high = 2 * mu_step_size * (1 * (dt1 - dt1_high));
	}

	if ((-dt2 + dt2_low) <= 0){
		AL_dpt_dt2_low = 0;
	}else if ((-dt2 + dt2_low) > 0){
		AL_dpt_dt2_low = 2 * mu_step_size * (-1 * (-dt2 + dt2_low));
	}
	if ((dt2 - dt2_high) <= 0){
		AL_dpt_dt2_high = 0;
	}else if ((dt2 - dt2_high) > 0){
		AL_dpt_dt2_high = 2 * mu_step_size * (1 * (dt2 - dt2_high));
	}

	if ((px1u1/w[6]) <= 0){
		AL_dpx1u1_dt0 = 0;
	}else if ((px1u1/w[6]) > 0){
		AL_dpx1u1_dt0 = 2 * mu_step_size * (px1u1/w[6]) * dpx1u1_dt0/w[6];
	}

	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_dt0 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_dt0 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_dt0/w[7];
	}

	if ((px2u2/w[7]) <= 0){
		AL_dpx2u2_dt1 = 0;
	}else if ((px2u2/w[7]) > 0){
		AL_dpx2u2_dt1 = 2 * mu_step_size * (px2u2/w[7]) * dpx2u2_dt1/w[7];
	}

	if ((px2u1/w[8]) <= 0){
		AL_dpx2u1_dt0 = 0;
	}else if ((px2u1/w[8]) > 0){
		AL_dpx2u1_dt0 = 2 * mu_step_size * (px2u1/w[8]) * dpx2u1_dt0/w[8];
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_dt0 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_dt0 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_dt0/w[9];
	}

	if ((px2u1/w[8]) <= 0){
		AL_dpx2u1_dt1 = 0;
	}else if ((px2u1/w[8]) > 0){
		AL_dpx2u1_dt1 = 2 * mu_step_size * (px2u1/w[8]) * dpx2u1_dt1/w[8];
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_dt1 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_dt1 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_dt1/w[9];
	}

	if ((px3u2/w[9]) <= 0){
		AL_dpx3u2_dt2 = 0;
	}else if ((px3u2/w[9]) > 0){
		AL_dpx3u2_dt2 = 2 * mu_step_size * (px3u2/w[9]) * dpx3u2_dt2/w[9];
	}

	double AL_dt0_inequality_penalties = AL_dpt_dt0_low + AL_dpt_dt0_high + AL_dpx1u1_dt0 + AL_dpx2u2_dt0 + AL_dpx2u1_dt0 + AL_dpx3u2_dt0;
	double AL_dt1_inequality_penalties = AL_dpt_dt1_low + AL_dpt_dt1_high + AL_dpx2u2_dt1 + AL_dpx2u1_dt1 + AL_dpx3u2_dt1;
	double AL_dt2_inequality_penalties = AL_dpt_dt2_low + AL_dpt_dt2_high + AL_dpx3u2_dt2;

//    cout<< "dt inequality constraint" << endl;

//    cout << "Jacobian dt element " << endl;
//    cout << dpt_dt0 << endl;
//    cout << dpt_dt1 << endl;
//    cout << dpt_dt2 << endl;
//    cout << dpx1u1_dt0 << endl;
//    cout << dpx2u1_dt0 << endl;
//    cout << dpx2u2_dt0 << endl;
//    cout << dpx3u2_dt0 << endl;
//    cout << dpx2u1_dt1 << endl;
//    cout << dpx2u2_dt1 << endl;
//    cout << dpx3u2_dt1 << endl;
//    cout << dpx3u2_dt2 << endl;
//    cout << dpdy3dy3_dt2 << endl;

    Vector3d dJ_dt_;

//    cout << "dJ with dt gradient" << endl;
//    cout << dJ_dt0(dt_s, u_s, x_s, xdot_s, xddot_s) << endl;
//    cout << dJ_dt1(dt_s, u_s, x_s, xdot_s, xddot_s) << endl;
//    cout << dJ_dt2(dt_s, u_s, x_s, xdot_s, xddot_s) << endl;

    dJ_dt_ << dJ_dt0(dt_s, u_s, x_s, xdot_s, xddot_s) + dt0_inequality_penalties + AL_dt0_inequality_penalties, //
              dJ_dt1(dt_s, u_s, x_s, xdot_s, xddot_s) + dt1_inequality_penalties + AL_dt1_inequality_penalties, //
              dJ_dt2(dt_s, u_s, x_s, xdot_s, xddot_s) + dt2_inequality_penalties + AL_dt2_inequality_penalties; //

    return dJ_dt_;
}