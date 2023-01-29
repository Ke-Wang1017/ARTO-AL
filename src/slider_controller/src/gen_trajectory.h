/******************************************************************************
-Author: Zhonghe Jiang (Harry) and Ke Wang
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#ifndef GEN_TRAJECTORY_H_
#define GEN_TRAJECTORY_H_

#include <iostream>
#include "define.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


/**
* @brief 3rd and 5th order polynomial interpolation
*/
Vector3d thirdOrderPolynomialInterpolation (double tDuration, double tCurrent, double startPos, double startVel, double startAcc, double endPos, double endVel, double endAcc);
Vector3d fifthOrderPolynomialInterpolation (double tDuration, double tCurrent, double startPos, double startVel, double startAcc, double endPos, double endVel, double endAcc);

void genLIPM (Matrix2d &A, Vector2d &B, double Ts);


#endif /* GEN_TRAJECTORY_H_ */