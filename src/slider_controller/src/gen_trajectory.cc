/******************************************************************************
-Author: Zhonghe Jiang (Harry) and Ke Wang
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#include "gen_trajectory.h"
#include <cmath>


Vector3d fifthOrderPolynomialInterpolation(double tDuration, double tCurrent, double startPos, double startVel, double startAcc, double endPos, double endVel, double endAcc)
{ 
  VectorXd coeffs(6);  
  coeffs(0) = -0.5*(12*startPos-12*endPos+6*startVel*tDuration+6*endVel*tDuration+startAcc*pow(tDuration,2)-endAcc*pow(tDuration,2))/pow(tDuration,5);
  coeffs(1) = 0.5*(30*startPos-30*endPos+16*startVel*tDuration+14*endVel*tDuration+3*startAcc*pow(tDuration,2)-2*endAcc*pow(tDuration,2))/pow(tDuration,4);
  coeffs(2) = -0.5*(20*startPos-20*endPos+12*startVel*tDuration+8*endVel*tDuration+3*startAcc*pow(tDuration,2)-endAcc*pow(tDuration,2))/pow(tDuration,3);
  coeffs(3) = 0.5*startAcc;
  coeffs(4) = startVel;
  coeffs(5) = startPos;


  double position = coeffs(0)*pow(tCurrent,5)+coeffs(1)*pow(tCurrent,4)+coeffs(2)*pow(tCurrent,3)+coeffs(3)*pow(tCurrent,2)+coeffs(4)*pow(tCurrent,1)+coeffs(5);
  double velocity = 5*coeffs(0)*pow(tCurrent,4)+4*coeffs(1)*pow(tCurrent,3)+3*coeffs(2)*pow(tCurrent,2)+2*coeffs(3)*pow(tCurrent,1)+coeffs(4);
  double acceleration = 20*coeffs(0)*pow(tCurrent,3)+12*coeffs(1)*pow(tCurrent,2)+6*coeffs(2)*pow(tCurrent,1)+2*coeffs(3);
  Vector3d nextTraj; 
  nextTraj << position, velocity, acceleration;

  // cout << "---------------------------------" << endl;
  // cout << "Coeficients: " << coeffs << endl;
  // cout << "tDuration: " << tDuration << endl;
  // cout << "tCurrent: " << tCurrent << endl;
  // cout << "startPos: " << startPos << endl;
  // cout << "startVel: " << startVel << endl;
  // cout << "startAcc: " << startAcc << endl;
  // cout << "endPos: " << endPos << endl;
  // cout << "endVel: " << endVel << endl;
  // cout << "endAcc: " << endAcc << endl;
  // cout << "nextTraj: \n" << nextTraj << endl;

  return nextTraj;
}


Vector3d thirdOrderPolynomialInterpolation(double tDuration, double tCurrent, double startPos, double startVel, double startAcc, double endPos, double endVel, double endAcc)
{ 
  VectorXd coeffs(4);  
  coeffs(0) = startPos;
  coeffs(1) = startVel;
  coeffs(2) = (endPos - startPos - (tDuration*endVel/3) - 2*startVel*tDuration/3)/(tDuration*tDuration/3);
  coeffs(3) = (endPos - startPos - (tDuration*endVel/2) - 1*startVel*tDuration/2)/(-tDuration*tDuration*tDuration/2);

  double position = coeffs(0) + coeffs(1)*tCurrent + coeffs(2)*pow(tCurrent,2) + coeffs(3)*pow(tCurrent,3);
  double velocity = coeffs(1) + 2*coeffs(2)*tCurrent + 3*coeffs(3)*pow(tCurrent,2);
  double acceleration = 2*coeffs(2) + 6*coeffs(3)*tCurrent;
//  cout << "current position " << startPos << endl;
//  cout << "next position " << position << endl;
//  cout << "current velocity " << startVel << endl;
//  cout << "next velocity " << velocity << endl;
  Vector3d nextTraj; 
  nextTraj << position, velocity, acceleration;

  // cout << "---------------------------------" << endl;
  // cout << "Coeficients: " << coeffs << endl;
  // cout << "tDuration: " << tDuration << endl;
  // cout << "tCurrent: " << tCurrent << endl;
  // cout << "startPos: " << startPos << endl;
  // cout << "startVel: " << startVel << endl;
  // cout << "startAcc: " << startAcc << endl;
  // cout << "endPos: " << endPos << endl;
  // cout << "endVel: " << endVel << endl;
  // cout << "endAcc: " << endAcc << endl;
  // cout << "nextTraj: \n" << nextTraj << endl;

  return nextTraj;
}


void genLIPM (Matrix2d &A, Vector2d &B, double Ts)
{
  double w = sqrt(GRAVITY/com_height);
  double ch = cosh(w*Ts);
  double sh = sinh(w*Ts);
  A << ch, sh/w, w*sh, ch;
  B << 1-ch ,-w*sh;
}