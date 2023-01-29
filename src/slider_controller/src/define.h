/******************************************************************************
-Author: Zhonghe Jiang (Harry)
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#ifndef DEFINE_H_
#define DEFINE_H_

#include <cmath>

#define LOOP_RATE 1000 // loop rate for the controller
#define GRAVITY 9.8
#define PI 3.14159

enum Support { L = -1, D, R, S};

// Initial pelvis height
const double pelvis_height = 0.7;

// CoM height
const double com_height = 0.69;

// Height of the end-effector
// if ROTO gait, then -0.001
//const double foot_height = -0.001;

// Foot width
const double foot_length = 0.2;

// if SLIDER with FOOT, then 0.04
const double foot_height = 0.038;

// Peak of the swing foot
// if ROTO gait, then 0.04
//const double step_height = 0.04;

// if SLIDER with FOOT, then 0.08
const double step_height = 0.1;

// Step width
const double step_width = 0.4;
// Time required for the initial sway
const double initial_sway_time = 0.7;

// Time delay before updating robot configuration from Gazebo data
const double update_delay = 10;

const double k_LIPM = sqrt(com_height/GRAVITY);


#endif /* DEFINE_H_ */
