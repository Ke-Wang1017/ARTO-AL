/******************************************************************************
-Author: Zhonghe Jiang and Ke Wang
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#ifndef LIPM_H_
#define LIPM_H_

#include <iostream>
#include <cmath>

using namespace std;


/**
* @brief Based on the analytical solution of the linear inverted pendulum model, without foot placement
*/
double calc_Pos1_given_TPos0Vel0(double Tc, double t, double pos0, double vel0)
{
	return pos0*cosh(t/Tc) + Tc*vel0*sinh(t/Tc);
}

double calc_Vel0_given_TPos0Vel1(double Tc, double t, double pos0, double vel1)
{
	return (vel1 - pos0*sinh(t/Tc)/Tc ) / cosh(t/Tc);
}




#endif /* LIPM_H_ */