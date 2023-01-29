/******************************************************************************
-Author: Ke Wang & Zhonghe Jiang (Harry)
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#include "osc_foot.h"

void OSC::updateData (const pin::Model &model, pin::Data &data,
                      const VectorXd &q_in, const VectorXd &v_in, const Support whichLeg, int counter) 
{
  nq = model.nq;
  nv = model.nv;
  q = q_in;
  v = v_in;
  // cout << "q = " << q.transpose() << endl;
  // cout << "v = " << v.transpose() << endl;
  pin::forwardKinematics(model,data,q);

  // Computes global COM, COM Jacobian and joint Jacobian. 
  pin::jacobianCenterOfMass(model,data,q,false); 
  // cout << "------------COM jacobian------------" << endl;
  // cout << data.Jcom << endl;
  Jcom = data.Jcom;

  if (counter == 0)
  {
    Jcom_last = Jcom;
  }
  dJcom = (Jcom - Jcom_last)/0.001;
  Jcom_last = Jcom;

  // These two functions have to be called before running getFrameJacobian
  pin::computeJointJacobians(model,data,q);
  pin::framesForwardKinematics(model,data,q);

  // Centroidal Momentum Matrix
  pin::computeCentroidalMap(model, data, q);
  Ag = data.Ag;
  pin::computeCentroidalMapTimeVariation(model, data, q, v);
  dAg = data.dAg;

  // cout << "------------Pelvis jacobian------------" << endl;
  // // because the Jacobian returns the previous frame jacobian, so we use left_contact_1 to get the real foot Jacobian
  MatrixXd J_pelvis = MatrixXd::Zero(6,nv);
  pin::getFrameJacobian(model,data,model.getFrameId("base_link"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED,J_pelvis);
  // cout<< "Base Jacobian is\n" << J_pelvis << endl;
  J_pelvis_angular = J_pelvis.block(3,0,3,nv);

  // cout << "------------Left foot jacobian------------" << endl;
  J_left = MatrixXd::Zero(6,nv);
  pin::getFrameJacobian(model,data,model.getFrameId("Left_Foot"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED,J_left);
  // pin::getFrameJacobian(model,data,model.getFrameId("left_contact1"),pin::ReferenceFrame::WORLD,J_left);
  // J_left_linear = J_left.block(0,0,3,nv);


  // cout << "------------Right foot jacobian------------" << endl;
  J_right = MatrixXd::Zero(6,nv);
  pin::getFrameJacobian(model,data,model.getFrameId("Right_Foot"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED,J_right);
  // pin::getFrameJacobian(model,data,model.getFrameId("right_contact1"),pin::ReferenceFrame::WORLD,J_right);
  // J_right_linear = J_right.block(0,0,3,nv);


  // cout << "------------Left foot pad jacobian------------" << endl;
  MatrixXd J_left_pad1 = MatrixXd::Zero(6,nv);
  MatrixXd J_left_pad2 = MatrixXd::Zero(6,nv);
  MatrixXd J_left_pad3 = MatrixXd::Zero(6,nv);
  MatrixXd J_left_pad4 = MatrixXd::Zero(6,nv);
  pin::getFrameJacobian(model,data,model.getFrameId("Left_Foot_A"),pin::ReferenceFrame::LOCAL,J_left_pad1);
  pin::getFrameJacobian(model,data,model.getFrameId("Left_Foot_B"),pin::ReferenceFrame::LOCAL,J_left_pad2);
  pin::getFrameJacobian(model,data,model.getFrameId("Left_Foot_C"),pin::ReferenceFrame::LOCAL,J_left_pad3);
  pin::getFrameJacobian(model,data,model.getFrameId("Left_Foot_D"),pin::ReferenceFrame::LOCAL,J_left_pad4);
  // pin::getFrameJacobian(model,data,model.getFrameId("left_pad1"),pin::ReferenceFrame::WORLD,J_left_pad1);
  // pin::getFrameJacobian(model,data,model.getFrameId("left_pad2"),pin::ReferenceFrame::WORLD,J_left_pad2);
  // pin::getFrameJacobian(model,data,model.getFrameId("left_pad3"),pin::ReferenceFrame::WORLD,J_left_pad3);
  // pin::getFrameJacobian(model,data,model.getFrameId("left_pad4"),pin::ReferenceFrame::WORLD,J_left_pad4);
  MatrixXd J_left_pad1_linear = J_left_pad1.block(0,0,3,nv);
  MatrixXd J_left_pad2_linear = J_left_pad2.block(0,0,3,nv);
  MatrixXd J_left_pad3_linear = J_left_pad3.block(0,0,3,nv);
  MatrixXd J_left_pad4_linear = J_left_pad4.block(0,0,3,nv);
  // cout << J_left_pad1_linear << endl;
  // cout << J_left_pad2_linear << endl;
  // cout << J_left_pad3_linear << endl;
  // cout << J_left_pad4_linear << endl;



  // cout << "------------Right foot pad jacobian------------" << endl;
  MatrixXd J_right_pad1 = MatrixXd::Zero(6,nv);
  MatrixXd J_right_pad2 = MatrixXd::Zero(6,nv);
  MatrixXd J_right_pad3 = MatrixXd::Zero(6,nv);
  MatrixXd J_right_pad4 = MatrixXd::Zero(6,nv);
  pin::getFrameJacobian(model,data,model.getFrameId("Right_Foot_A"),pin::ReferenceFrame::LOCAL,J_right_pad1);
  pin::getFrameJacobian(model,data,model.getFrameId("Right_Foot_B"),pin::ReferenceFrame::LOCAL,J_right_pad2);
  pin::getFrameJacobian(model,data,model.getFrameId("Right_Foot_C"),pin::ReferenceFrame::LOCAL,J_right_pad3);
  pin::getFrameJacobian(model,data,model.getFrameId("Right_Foot_D"),pin::ReferenceFrame::LOCAL,J_right_pad4);
  // pin::getFrameJacobian(model,data,model.getFrameId("right_pad1"),pin::ReferenceFrame::WORLD,J_right_pad1);
  // pin::getFrameJacobian(model,data,model.getFrameId("right_pad2"),pin::ReferenceFrame::WORLD,J_right_pad2);
  // pin::getFrameJacobian(model,data,model.getFrameId("right_pad3"),pin::ReferenceFrame::WORLD,J_right_pad3);
  // pin::getFrameJacobian(model,data,model.getFrameId("right_pad4"),pin::ReferenceFrame::WORLD,J_right_pad4);
  MatrixXd J_right_pad1_linear = J_right_pad1.block(0,0,3,nv);
  MatrixXd J_right_pad2_linear = J_right_pad2.block(0,0,3,nv);
  MatrixXd J_right_pad3_linear = J_right_pad3.block(0,0,3,nv);
  MatrixXd J_right_pad4_linear = J_right_pad4.block(0,0,3,nv);
  // cout << J_right_pad1_linear << endl;
  // cout << J_right_pad2_linear << endl;
  // cout << J_right_pad3_linear << endl;
  // cout << J_right_pad4_linear << endl;



  // cout << "------------Combined support jacobian------------" << endl;
  n_ee = 2; // Number of end-effectors
  n_contact = 4; // Number of contact points per foot
  np = n_ee*n_contact; // Total number of contact points

  // The combined support jacobiean stack the end-effctor Jacobian vertically for each support point
  MatrixXd Jc(3*np, nv);
  Jc << J_left_pad1_linear,
        J_left_pad2_linear,
        J_left_pad3_linear,
        J_left_pad4_linear,
        J_right_pad1_linear,
        J_right_pad2_linear,
        J_right_pad3_linear,
        J_right_pad4_linear;
  Jc_T = Jc.transpose();
  // cout << Jc_T << endl;

  // Computes joint Jacobian time derivative
  pin::computeJointJacobiansTimeVariation(model,data,q,v);
  
  // cout << "------------Left foot jacobian time derivative------------" << endl;
  dJ_left = MatrixXd::Zero(6,nv);
  pin::getFrameJacobianTimeVariation(model,data,model.getFrameId("Left_Foot"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_left);
  // pin::getFrameJacobianTimeVariation(model,data,model.getFrameId("left_contact1"),pin::ReferenceFrame::WORLD, dJ_left);
  // dJ_left_linear = dJ_left.block(0,0,3,nv);
  // cout << dJ_left << endl;
  // cout << dJ_left_linear << endl;
  // cout << "------------Right foot jacobian time derivative------------" << endl;
  dJ_right = MatrixXd::Zero(6,nv);
  pin::getFrameJacobianTimeVariation(model,data,model.getFrameId("Right_Foot"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_right);
  // pin::getFrameJacobianTimeVariation(model,data,model.getFrameId("right_contact1"),pin::ReferenceFrame::WORLD, dJ_right);
  // dJ_right_linear = dJ_right.block(0,0,3,nv);
  // cout << dJ_right << endl;
  // cout << dJ_right_linear << endl;
  
  MatrixXd dJ_pelvis = MatrixXd::Zero(6,nv);
  pin::getFrameJacobianTimeVariation(model,data,model.getFrameId("base_link"),pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_pelvis);
  dJ_pelvis_angular = dJ_pelvis.block(3,0,3,nv);


  // Computes COM position, velocity
  pin::centerOfMass(model,data,q,v,false);
  // cout << "------------COM position------------" << endl;
  // cout << data.com[0] << endl;
  // cout << "------------COM velocity------------" << endl;
  // cout << data.vcom[0] << endl;

  // Computes the upper triangular part of the joint space inertia matrix M
  pin::crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // cout << "------------Joint space inertia matrix M------------" << endl;
  // cout << data.M << endl;
  M = data.M;

  // Computes the nonlinear effects of the Lagrangian dynamics
  pin::nonLinearEffects(model,data,q,v);
  // cout << "------------Nonlinear effects of the Lagrangian dynamics------------" << endl;
  // cout << data.nle << endl;
  nle = data.nle;

  // Construct the actuated joint selection matrix, its transpose should be [zero; identity]
  n_actuator = nv-6;
  Sa_T = MatrixXd::Zero(nv, n_actuator);
  Sa_T.block(6,0,n_actuator, n_actuator) = MatrixXd::Identity(n_actuator, n_actuator);
  // cout << "------------Transpose of the actuated joint selection matrix (16*10 for SLIDER)------------" << endl;
  // cout << Sa_T << endl;
  // Construct the contact force selection matrix, its transpose should be [zero; identity]
  Sf_T = MatrixXd::Zero(3*np, 3*np);
  const int dim_fi = 3*n_contact; // Dimension of the force vector per foot
  switch(whichLeg)
  {
    case Support::L : Sf_T.block(dim_fi,dim_fi,dim_fi,dim_fi) = MatrixXd::Identity(dim_fi, dim_fi);
                      break;
    case Support::R : Sf_T.block(0,0,dim_fi,dim_fi) = MatrixXd::Identity(dim_fi, dim_fi);
                      break;
    case Support::D : break;
  }  
  // cout << "------------Transpose of the contact force selection matrix (24*24 for SLIDER)------------" << endl;
  // cout << Sf_T << endl;

}


VectorXd OSC::solveQP (const Vector3d &ddx_com, const Vector3d &dh_ang, const Vector3d &ddx_left, const Vector3d &ddx_right, const Vector3d &ddx_left_ori, const Vector3d &ddx_right_ori, const Vector3d &ddx_base_orientation, const VectorXd &w, const double k, const VectorXd &torque_limit, int counter)
{
  // Weighting matrix in the objective function
  MatrixXd W(21, 21);
  W = w.asDiagonal();

  // Construct QP objective function
  MatrixXd A(21, nv);
  A << Jcom,
       Ag.block(3, 0, 3, nv),
       J_left,
       J_right,
       J_pelvis_angular;

  MatrixXd dA(21, nv);
  dA << dJcom,
        dAg.block(3, 0, 3, nv),
        dJ_left,
        dJ_right,
        dJ_pelvis_angular;

  MatrixXd ddx_cmd(21, 1);
  ddx_cmd << ddx_com,
             dh_ang,
             ddx_left,
             ddx_left_ori,
             ddx_right,
             ddx_right_ori,
             ddx_base_orientation;

  MatrixXd H0 = 2*A.transpose()*W*A;
  VectorXd g0 = 2*A.transpose()*W*(dA*v - ddx_cmd);
 
  // cout << "W: \n" << W << endl; 
  // cout << "v = " << v.transpose() << endl;
  // cout << "Jcom: \n" << Jcom << endl; 
  // cout << "J_left_linear: \n" << J_left_linear << endl; 
  // cout << "J_right_linear: \n" << J_right_linear << endl; 
  // cout << "dJcom: \n" << dJcom << endl; 
  // cout << "dJ_left_linear: \n" << dJ_left_linear << endl; 
  // cout << "dJ_right_linear: \n" << dJ_right_linear << endl; 
   
  // cout << "CoM Jacobian:\n" << Jcom << endl;
  // cout << "---------------------------" << endl;
  // cout << "ddx_com: \n" << ddx_com << endl;
  // cout << "ddx_left: \n" << ddx_left << endl;
  // cout << "ddx_right: \n" << ddx_right << endl;
  // cout << "ddx_right: \n" << ddx_right << endl;
  // cout << "ddx_base_orientation: \n" << ddx_base_orientation << endl;
  // cout << "---------------------------" << endl;

  MatrixXd H = MatrixXd::Zero(n_variables,n_variables); // Hessian matrix of the QP
  H.block(0,0,nv,nv) = H0;
  H = H + 0.0001*MatrixXd::Identity(n_variables,n_variables); // This trick makes H positve definite

  VectorXd g = VectorXd::Zero(n_variables,1); // Gradient vector of the QP
  g.block(0,0,nv,1) = g0;


  // Construct QP constraints
  MatrixXd B(4, 3);
  B << 1,  0, -k,
      -1,  0, -k,
       0,  1, -k,
       0, -1, -k;

  MatrixXd Bt = MatrixXd::Zero(4*np, 3*np);
  for (int i=0; i<np; i++)
  {
    Bt.block(4*i,3*i,4,3) = B; 
  } 

  MatrixXd A_QP = MatrixXd::Zero(n_constraints,n_variables); 
  A_QP.block(0,0,nv,nv) = M; 
  A_QP.block(0,nv,nv,3*np) = -Jc_T; 
  A_QP.block(nv,nv,4*np,3*np) = Bt; 
  A_QP.block(nv+4*np,nv,3*np,3*np) = Sf_T; 


  // Lower bounds of the constraints
  VectorXd lbA = VectorXd::Zero(n_constraints,1);
  lbA.block(0,0,nv,1) = -nle;
  lbA.block(6,0,n_actuator,1) -= torque_limit;
  lbA.block(nv,0,4*np,1) = -inf*VectorXd::Ones(4*np,1);

  VectorXd lb = -inf*VectorXd::Ones(n_variables,1);
  for(int i=0; i<np; i++)
  {
    lb(nv + 2 + 3*i) = 0;
  }

  // Upper bounds of the constraints
  VectorXd ubA = VectorXd::Zero(n_constraints,1);
  ubA.block(0,0,nv,1) = -nle;
  ubA.block(6,0,n_actuator,1) += torque_limit;

  VectorXd ub = inf*VectorXd::Ones(n_variables,1);


  // Constrain base motion
  // lb[0] = -1; // base x
  // ub[0] = 1; // base x

  // lb[1] = -2; // base y
  // ub[1] = 2; // base y

  // lb[2] = -2; // base z
  // ub[2] = 2; // base z

  lb[3] = -1.0; // base roll
  ub[3] = 1.0; // base roll

  lb[4] = -1.5; // base pitch
  ub[4] = 1.5; // base pitch

  lb[5] = -0.5; // base yaw
  ub[5] = 0.5; // base yaw

  // lb[6] = -20; // left hip roll
  // ub[6] = 20; // left hip roll

  // lb[7] = -20; // left hip pitch
  // ub[7] = 20; // left hip pitch

  // lb[8] = -20; // left hip slide
  // ub[8] = 20; // left hip slide

  // lb[9] = -30; // left ankle pitch
  // ub[9] = 30; // left ankle pitch

  // lb[10] = -30; // left ankle roll
  // ub[10] = 30; // left ankle roll

  // lb[11] = -20; // right hip roll
  // ub[11] = 20; // right hip roll

  // lb[12] = -20; // right hip pitch
  // ub[12] = 20; // right hip pitch

  // lb[13] = -20; // right hip slide
  // ub[13] = 20; // right hip slide

  // lb[14] = -30; // right ankle pitch
  // ub[14] = 30; // right ankle pitch

  // lb[15] = -30; // right ankle roll
  // ub[15] = 30; // right ankle roll




  // cout << "------------M------------" << endl;
  // cout << M << endl;
  // cout << "------------Sa_T------------" << endl;
  // cout << Sa_T << endl;
  // cout << "------------Jc_T------------" << endl;
  // cout << Jc_T << endl;
  // cout << "------------Bt------------" << endl;
  // cout << Bt << endl;
  // cout << "------------Sf_T------------" << endl;
  // cout << Sf_T << endl;
  // cout << "------------nle------------" << endl;
  // cout << nle << endl;


  // cout << "------------Hessian Matrix (40*40 for SLIDER)------------" << endl;
  // cout << H << endl;
  // cout << "------------H0------------" << endl;
  // cout << H0 << endl;
  // cout << "------------Gradient vector (40*1 for SLIDER)------------" << endl;
  // cout << g << endl;
  // cout << "------------Bt------------" << endl;
  // cout << Bt << endl;
  // cout << "------------A_QP (72*40 for SLIDER)------------" << endl;
  // cout << A_QP << endl;
  // cout << "------------lbA (72*1 for SLIDER)------------" << endl;
  // cout << lbA << endl;
  // cout << "------------ubA (72*1 for SLIDER)------------" << endl;
  // cout << ubA << endl;
  // cout << "------------lb (40*1 for SLIDER)------------" << endl;
  // cout << lb << endl;
  // cout << "------------ub (40*1 for SLIDER)------------" << endl;
  // cout << ub << endl;


  // cout << "-----------check PD of H--------------" << endl;
  // Eigen::LLT<Eigen::MatrixXd> lltOfH(H); // compute the Cholesky decomposition of H
  // if(lltOfH.info() == Eigen::NumericalIssue)
  // {
  //   cout << "Not PD !!!!!!!!!!" << endl;
  // }
  // else
  // {
  //   cout << "It is PD ^-^" << endl;
  // }

  // cout << "-----------check isInvertible of A--------------" << endl;
  // Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> lu(A);
  // if(lu.isInvertible())
  // {
  //   cout << "It is invertible ^-^" << endl;
  // }
  // else
  // {
  //   cout << "It is not invertible!!!!!!" << endl;
  // }


  // Solving QP

  /* Setup data for the QP. */
  real_t H_array[n_variables*n_variables];
  real_t A_array[n_constraints*n_variables];
  real_t g_array[n_variables];
  real_t lb_array[n_variables];
  real_t ub_array[n_variables];
  real_t lbA_array[n_constraints];
  real_t ubA_array[n_constraints];


  RowMajMat::Map(H_array, H.rows(), H.cols()) = H;
  RowMajMat::Map(A_array, A_QP.rows(), A_QP.cols()) = A_QP;
  RowMajMat::Map(g_array, g.rows(), g.cols()) = g;
  RowMajMat::Map(lb_array, lb.rows(), lb.cols()) = lb;
  RowMajMat::Map(ub_array, ub.rows(), ub.cols()) = ub;
  RowMajMat::Map(lbA_array, lbA.rows(), lbA.cols()) = lbA;
  RowMajMat::Map(ubA_array, ubA.rows(), ubA.cols()) = ubA;

  // cout << "-----------------check row major-------------------" << endl;
  // cout << "H: \n" << H << endl;
  // cout << "H_array: \n" << endl; 
  // for (int i=0; i<100; i++)
  // {
  //   cout << H_array[i] << endl;
  // }

  
  
  /* Solve QP. */
  int_t nWSR = 10000; // Maximum number of working set recalculations
  returnValue returnValue;

  Options myOptions;
  // myOptions.setToReliable( );
  myOptions.setToMPC( );
  myOptions.printLevel = PL_LOW;
  // myOptions.enableNZCTests = BT_TRUE;
  // myOptions.enableFlippingBounds = BT_TRUE;
  // myOptions.initialStatusBounds = ST_UPPER;
  qp.setOptions( myOptions );

  if(counter == 0)
  {
    // cout << "--------------Construct initial guess-----------------" << endl;
    // VectorXd a = VectorXd::Zero(16);
    // VectorXd tau_des = VectorXd::Zero(10);
    // VectorXd f_test = VectorXd::Zero(24);
    // VectorXd xOpt_guess(n_variables);
    // xOpt_guess << a,
    //               tau_des,
    //               f_test;
    // real_t xOpt_guess_array[n_variables];
    // RowMajMat::Map(xOpt_guess_array, xOpt_guess.rows(), xOpt_guess.cols()) = xOpt_guess;
    // for(int i=0; i<n_variables; i++)
    // {
    //   cout << xOpt_guess[i] << endl;
    // }
    // cout << "objective value of initial guess: \n" << 0.5*xOpt_guess.transpose()*H*xOpt_guess << endl;

    // cout << "test initial guess: \n" << (M*a + nle - Sa_T*tau_des - Jc_T*f_test) << endl;

    // int_t nV = qp.getNV( );
    // int_t nC = qp.getNC( );
    // Bounds guessedBounds( nV );
    // guessedBounds.setupAllFree( );
    // Constraints guessedConstraints( nC );
    // guessedConstraints.setupAllUpper( );
    // for(int i=nv; i<(nv+4*np); i++)
    // {
    //   guessedConstraints.moveActiveToInactive(i);  
    // }

    // returnValue = qp.init( H_array,g_array,A_array,lb_array,ub_array,lbA_array,ubA_array, nWSR,0, xOpt_guess_array, NULL ,&guessedBounds, &guessedConstraints, NULL);
    // returnValue = qp.init( H_array,g_array,A_array,lb_array,ub_array,lbA_array,ubA_array, nWSR,0, xOpt_guess_array);
    returnValue = qp.init( H_array,g_array,A_array,lb_array,ub_array,lbA_array,ubA_array, nWSR,0 );
  }
  else
  {
    returnValue = qp.hotstart( H_array,g_array,A_array,lb_array,ub_array,lbA_array,ubA_array, nWSR,0 );
  }

  // cout << "Slover status: " << getSimpleStatus(returnValue) << endl << flush;

  /* Get and print solution of the QP. */
  VectorXd tau;
  real_t xOpt[n_variables];
  qp.getPrimalSolution( xOpt );

  // cout << "\n**************Optimal Solution****************" << endl;
  // cout << "*************Stacked as ddq (16), f (24)***************" << endl;
  VectorXd qdd_Opt = Map<VectorXd>(xOpt, 16);
  VectorXd f_Opt = Map<VectorXd>((xOpt+16), 24);

  tau = M.bottomRows(n_actuator)*qdd_Opt + nle.bottomRows(n_actuator) - Jc_T.bottomRows(n_actuator)*f_Opt;

  // cout << "base orientation task:\n" << J_pelvis_angular*qdd_Opt + dJ_pelvis_angular*v - ddx_base_orientation << endl;
  // cout << "Force:\n" << f_Opt << endl;
  // cout << "Torque:" << endl;
  // cout << tau << endl;

  // cout << "optimal objective value: " << qp.getObjVal() << endl;


  return tau;

}