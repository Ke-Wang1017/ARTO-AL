#!/usr/bin/env python
"""
foot_step_planner.py
takes reference trajectory and current state, produces footstep positions
and times for next N steps.

Digby Chappell and Ke Wang
April 2020
"""

import time
import rospy
import numpy as np
from numpy import linalg as la
import sys
np.set_printoptions(threshold=sys.maxsize)
from casadi import *
from footstep_planner import SLIDER
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64

g = 9.81    # gravity
#
# const = [m, m_s, L, h]
#
# T_end = 100
# dt = 0.001
# vmax = 3             # m/s
# x_dot_ref = [0.4, 0.0]     # m/s
# weight_x = 1.0
# weight_y = 10.0
# weight_penalty_swing = 2
# weight_penalty_clearance = 2
# # Parameters subscribed from the slider_controller node #######################################
x = 0.0
x_dot = 0.0
x_dotdot = 0.0
y = 0.0
y_dot = 0.0
y_dotdot = 0.0
left_x = 0.00
left_y = 0.21
right_x = 0.0
right_y = -0.21
foot_flag = 1 # Right support
previous_foot_flag = -1 # Left support
current_time = 0.001
# foot_radius = 0.20
# foot_clearance_y = 0.25
# ###############################################################################################

# v_ref = np.array([0.2, 0.0])

def planner_callback(msg):
    global x, x_dot, x_dotdot, y, y_dot, y_dotdot, left_x, left_y, right_x, right_y, foot_flag, current_time
    x, x_dot, x_dotdot, y, y_dot, y_dotdot = msg.data[:6]
    left_x, left_y = msg.data[6:8]
    right_x, right_y = msg.data[8:10]
    foot_flag = msg.data[10]
    current_time = msg.data[11]
    # print('x', x, x_dot, x_dotdot)
    # print('y', y, y_dot, y_dotdot)

counter = 0

if __name__=="__main__":
    rospy.init_node('ros_SLIDER_planner_node', anonymous=True)
    pub = rospy.Publisher('/time_slider_gazebo/footstep_plan_rk4', Float64MultiArray, queue_size=1)
    pub_state = rospy.Publisher('/time_slider_gazebo/planner_input', Float64MultiArray, queue_size=1)
    planner_input = rospy.Subscriber('/time_slider_gazebo/planner_input', Float64MultiArray, planner_callback)
    pub_weight = rospy.Publisher('/time_slider_gazebo/cost_weight', Float64MultiArray, queue_size=1)
    # pub_Jacobian = rospy.Publisher('/time_slider_gazebo/jacobian', Float64MultiArray, queue_size=1)
    # pub_HessianInv = rospy.Publisher('/time_slider_gazebo/hessianInv', Float64MultiArray, queue_size=1)

    ######################################################################################################################################
    v_ref = np.array([0.0, 0.0])
    ######################################################################################################################################
    # for ROTO Gait, mass = 11.5, for slider with FOOT, mass = 15
    slider = SLIDER(mass=15, height=0.7, leg_length=1.2, foot_radius=0.2)
    slider.initialise(model=slider.full_dynamics,
                      cost=slider.step_velocity_cost_function,
                      footsteps=3, intervals=6)
    Q = np.array([[1, 0],
                  [0, 1]])
    slider.set_weightings(Q)
    w = Float64MultiArray()
    # J = Float64MultiArray()
    # Input = Float64MultiArray()
    # Hessian_msg = Float64MultiArray()
    rate=20
    r = rospy.Rate(rate)
    delta_t = 1/rate
    # initial values for optimal footsteps
    dT_opt = np.array([0.4, 0.4, 0.4])
    U_opt = np.array([[left_x, right_x, left_x],
                      [left_y, right_y, left_y]])
    # initial values for solver
    dT_i = dT_opt
    U_i = U_opt
    count = 0
    total_time = 0.0
    while not rospy.is_shutdown():
        start = time.time()
        if foot_flag == -1:
            Xs0 = [right_x, right_y]
            U0 = [left_x, left_y]
        elif foot_flag == 1:
            Xs0 = [left_x, left_y]
            U0 = [right_x, right_y]

        X0 = np.array([x, y, x_dot, y_dot])
        #X0 = slider.full_dynamics(X0, U0, delta_t, 5)
        dX0 = np.array([x_dot, y_dot, x_dotdot, y_dotdot])

        print('------------------------------------------------')
        print("COM position is ", X0[0], X0[1])
        print("Current foot position is ", U0[0], U0[1])
        print("Foot flag is ", foot_flag)
        print('------------------------------------------------')

        D0 = np.array([X0[0] + dX0[0]/slider.omega,
                       X0[1] + dX0[1]/slider.omega,
                       X0[2] + dX0[2]/slider.omega,
                       X0[3] + dX0[3]/slider.omega])

        #current_time += delta_t/2
        dt = current_time

        dxN = v_ref
        a_ref = (dxN - X0[2:])/1.2
        dcm_v_ref = v_ref + a_ref/slider.omega
        slider.set_parameters(X0, D0, dt, U0, Xs0, foot_flag, v_ref) # DCM: dcm_v_ref, RK4/FULL: v_ref

        # update initial values for next solver iteration
        if foot_flag != previous_foot_flag or (dT_opt[0]) < 5e-2:
            dT_i[:-1] = dT_opt[1:]
            dT_i[-1] = dT_opt[-1]

            U_i[:-1] = U_opt[1:]
            U_i[-1] = U_opt[-1] + (U_opt[-2] - U_opt[-1])
            previous_foot_flag = foot_flag
            pass
        else:
            dT_i = dT_opt - delta_t
            U_i = U_opt
        try:
            slider.set_initial_solution(dT_i, U_i)
            slider.solve()
            U_opt = slider.U_opt
            dT_opt = slider.dT_opt
            new_step = Float64MultiArray()
            new_step.layout.dim = [MultiArrayDimension('', 8, 1)]
            u_x_1 = U_opt[0, 1]
            u_y_1 = U_opt[1, 1]
            u_x_2 = U_opt[0, 2]
            u_y_2 = U_opt[1, 2]
            print('optimal U', U_opt)
            print('optimal T', dT_opt)
            # print('optimal X1', slider.sol.value(slider.x_next_1))
            # print('optimal X2', slider.sol.value(slider.x_next_2))
            # print('optimal X3', slider.sol.value(slider.x_next_3))
            # print('optimal dual weight', slider.W_opt)
            # g1 = slider.opti.g[9]
            # # dt1 = slider.dT[0]
            # Jacobian2 = slider.sol.value(jacobian(slider.opti.g, slider.dT))
            # Jacobian2 = Jacobian2.todense()
            # JacobianA = slider.sol.value(jacobian(slider.opti.g, slider.U))
            # JacobianA = JacobianA.todense()
            # JacobianB = slider.sol.value(jacobian(slider.opti.f + dot(slider.opti.lam_g, slider.opti.g), slider.U))
            # Jacobianx0_t = slider.sol.value(jacobian(slider.x_current_0, slider.dT))
            # x1_part = slider.x_next_1[0]
            # Jacobianx1_t = slider.sol.value(jacobian(x1_part, slider.U))
            # Jacobianx1_t = Jacobianx1_t.todense()
            # x2_part = slider.x_next_2[0]
            # Jacobianx2_t = slider.sol.value(jacobian(x2_part, slider.U))
            # Jacobianx2_t = Jacobianx2_t.todense()
            # x3_part = slider.x_next_3[0]
            # Jacobianx3_t = slider.sol.value(jacobian(x3_part, slider.U))
            # Jacobianx3_t = Jacobianx3_t.todense()

            # Jacobianf_t = slider.sol.value(jacobian(slider.opti.f, slider.dT))
            # Jacobianf_t = Jacobianf_t.todense()
            # x_part = slider.U[0,:]
            # Jacobianf_u = slider.sol.value(jacobian(slider.opti.f, slider.U))
            # Jacobianf_u = Jacobianf_u.todense()

            # JacobianC = slider.sol.value(jacobian(slider.opti.f + dot(slider.opti.lam_g, slider.opti.g), slider.X))
            # print("L from X\n", JacobianC)
            # Jacobian2 = Jacobian2.todense()
            # J_part2 = Jacobian2[12:,:]
            # J_part2 = Jacobian2
            # J_part2 = J_part2.todense()
            # x_val = slider.sol.value(slider.opti.x)
            # gg_val = slider.sol.value(slider.opti.g)

            # J_part = Jacobian[18:, :-9]
            # J_part = J_part.todense()
            # J_part = np.round(J_part, decimals=2)
            # J_part = J_part.flatten()
            # w.data = g_val
            # J.data = J_part.tolist()

            # Hessian = slider.sol.value(hessian(slider.opti.f + dot(slider.opti.lam_g, slider.opti.g), slider.opti.x)[0])  # Python
            # Hessian = Hessian.todense()
            # Hessian = Hessian[-7:, -7:]
            # Hessian_inv = np.linalg.inv(Hessian)
            # Hessian_inv = Hessian_inv.flatten().astype(np.float).tolist()
            # Hessian_msg.data = Hessian_inv[0]

            # Input.data = [x, x_dot+0.1, x_dotdot, y, y_dot-0.2, y_dotdot, left_x, left_y, right_x, right_y, foot_flag,
            #               current_time]
            g_val = slider.sol.value(slider.opti.lam_g)
            w.data = g_val
            # pub_state.publish(Input)
            if dT_opt[0] > 10/200:
                
                if counter < 10:
                    new_step.data = [u_x_1, u_y_1, u_x_2, u_y_2, dT_opt[0], dT_opt[1], dT_opt[2], foot_flag]
                    pub.publish(new_step)
                    pub_weight.publish(w)
                    counter += 1
                    print("publish !!!")
            else:
                dT_opt = np.array([dT_opt[1], dT_opt[2], dT_opt[2]])
                U_opt = np.array([[u_x_1, u_x_2, u_x_2 + (u_x_2 - u_x_1)],
                                  [u_y_1, u_y_2, u_y_1]])
                # initial values for solver
                dT_i = dT_opt
                U_i = U_opt
        except Exception as e:
            print('could not solve ')
            print(e)
            # initial values for optimal footsteps

            # if dT_i[0] > 5/250:
            #     u_x_1 = U_i[0, 1]
            #     u_y_1 = U_i[1, 1]
            #     u_x_2 = U_i[0, 2]
            #     u_y_2 = U_i[1, 2]
            #     new_step.data = [u_x_1, u_y_1, u_x_2, u_y_2, dT_i[0], dT_i[1], dT_i[2], foot_flag]
            #     pub.publish(new_step)
            #     print('publish Backup solution !!!!!!!!!!!!!!!!!!')
            # dT_opt = np.array([dT_opt[1], dT_opt[2], dT_opt[2]])
            # U_opt = np.array([[u_x_1, u_x_2, u_x_2+(u_x_2-u_x_1)],
            #                   [u_y_1, u_y_2, u_y_1]])

            # # initial values for solver
            # dT_i = dT_opt
            # U_i = U_opt

        end = time.time()
        print('time spent ', end-start)
        count += 1
        total_time += end-start
        print('average time is ', total_time/count)
        r.sleep()
