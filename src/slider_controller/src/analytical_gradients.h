#ifndef ANALYTICAL_GRADIENTS_H
#define ANALYTICAL_GRADIENTS_H
#include <cmath>
#include <Eigen/Dense>

double x(double dt, double u, double x_prev, double x_dot_prev);

double xdot(double dt, double u, double x_prev, double x_dot_prev);

double xddot(double dt, double u, double x_prev, double x_dot_prev);

// Analytical gradients
double dxdot_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int kt=0);

double dx_dt(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int kt=0);
double dx_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int ku=1);

double dxdot_du(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s, int kx=1, int ku=1);

double dJ_dux1(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s);

double dJ_dux2(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s);

double dJ_duy1(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s);
double dJ_duy2(Vector3d dt_s, Vector3d u_s, Vector4d x_s, Vector4d xdot_s, Vector4d xddot_s);
double dJ_dt0(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s);

double dJ_dt1(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s);

double dJ_dt2(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s);

Vector2d dJ_dux(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s);
 
Vector2d dJ_duy(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> x_dot_s, Matrix<double,2, 4> xd_dot_s);

Vector3d dJ_dt(Vector3d dt_s, Matrix<double,2, 3> u_s, Matrix<double,2, 4> x_s, Matrix<double,2, 4> xdot_s, Matrix<double, 2, 4> xddot_s);

#endif