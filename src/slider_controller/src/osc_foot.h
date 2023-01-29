/******************************************************************************
-Author: Ke Wang & Zhonghe Jiang (Harry)
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#ifndef OSC_FOOT_H_
#define OSC_FOOT_H_

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#define PINOCCHIO_WITH_URDFDOM

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/centroidal.hpp"

#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

#include "define.h"

namespace pin = pinocchio;
using namespace Eigen;
using namespace std;

// This is used to construct row major matrix at runtime
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> RowMajMat;


/**
 * @brief Operational space control for reference tracking.
 *
 * This class implements the operational space control.
 */
class OSC {
public:

  OSC () : qp(40,72) 
  {};

  /**
   * @brief Update the current configuration of the robot and compute the necessary matrices used in OSC
   * @param model        pinocchio::model obatined from urdf.
   * @param data         pinocchio::data obatined from model.
   * @param q_in         Joint position.
   * @param v_in         Joint velocity.
   * @param whichLeg     Support foot.
   * @param counter     Current time step.
   */
  void updateData (const pin::Model &model, pin::Data &data,
                   const VectorXd &q_in, const VectorXd &v_in, const Support whichLeg, const int counter);

  /**
   * @brief Solve the quadratic program
   * @param ddx_com      Desired center of mass acceleration.
   * @param ddx_left     Desired left foot acceleration.
   * @param ddx_right    Desired right foot acceleration.
   * @param w            Coefficients of the diagonal entrices of the weighting matrix W.
   * @param k            Ground friction coefficient.
   * @param torque_limit 
   * @param counter     Current time step.
   */
  VectorXd solveQP (const Vector3d &ddx_com, const Vector3d &dh_ang, const Vector3d &ddx_left, const Vector3d &ddx_right, const Vector3d &ddx_left_ori, const Vector3d &ddx_right_ori, const Vector3d &ddx_base_orientation, const VectorXd &w, const double k, const VectorXd &torque_limit, int counter);


  ~OSC ()
  {};


private:

  /** Parameters of the robot.
   */
  int nq, nv; // Dimension of q and dq.
  int n_ee, n_contact, np, n_actuator; // Number of end-effectors, contact points per foot and total # of contact points.

  /** Parameters for the QP.
   */
  double inf = 1000000;
  int n_variables = 40; // Number of variables for the QP
  int n_constraints = 72; // Number of inequality constraints for the QP

  /** Current joint position and velocity.
   */
  VectorXd q, v;

  /** Matrix depending on the current configuration of the robot.
   *  M is the mass matrix in the dynamics equation.
   *  nle is the nonlinear terms in the dynamics equation.
   *  Sa_T selects the actuated joints.
   *  Sf_T selects which foot is in contact with the ground.
   *  Jc_T is the Jacobian of the contact points on the robot with the ground.
   */
  MatrixXd M, nle, Sa_T, Sf_T, Jc_T;


  /** Jacobian of COM and its time derivative.
   */
  MatrixXd Jcom, dJcom, Jcom_last;


  /** Jacobian of the end-effectors and their time derivative.
   */
  MatrixXd J_left, J_right, dJ_left, dJ_right, J_right_linear, dJ_left_linear, dJ_right_linear, J_pelvis_angular, dJ_pelvis_angular;

  MatrixXd Ag, dAg;

  /* Setting up SQProblem object. */
  SQProblem qp;

};

#endif /* OSC_H_ */