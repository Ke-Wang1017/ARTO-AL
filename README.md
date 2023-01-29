# ARTO-AL
This is the code base of the paper "When and Where to Step: Terrain-Aware Real-Time Footstep Location and Timing Optimization for Bipedal Robots". This code contains the ARTO-AL optimizer, the whole-body controller for SLIDER robot and Gazebo simulation for SLIDER.
This code is tested on Ubuntu 18.04 with ROS melodic. 

The code requires installing Gazebo simulation, [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [Casadi](https://web.casadi.org/).
To run the code:
1. `catkin make`
2. source the ros and this repo.
3. `roslaunch time_slider_gazebo slider_controller.launch`

If you just want to use the optimizer, navigate the folder "src/slider_controller/src", "footstep_planner_2d.py" implememts IPOPT optimizer and "footstep_planner_BFGS.cpp" implements the analytical gradient optimizer.
