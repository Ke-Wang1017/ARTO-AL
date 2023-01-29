#!/usr/bin/env python
"""
foot_step_planner.py
takes reference trajectory and current state, produces footstep positions
and times for next N steps.

Digby Chappell and Ke Wang
April 2020
"""

import time
# import rospy
import numpy as np
from casadi import *
import matplotlib.pyplot as plt
# from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64

# constants
g = 9.81


class SLIDER:
    def __init__(self, mass, height, leg_length, foot_radius):
        """
        :param mass: mass of robot (kg, float)
        :param height: height of robot CoM (m, float)
        :param leg_length: length of robot leg (m, float)
        :param foot_radius: radius of the foot (m, float)
        """
        self.mass = mass
        self.h = height
        self.L = leg_length
        self.foot_r = foot_radius
        self.omega = np.sqrt(g / self.h)
        self.opti = Opti()
        self.solve_time = 0

    def initialise(self, model, cost, footsteps=3, intervals=5):
        """
        :param model: dynamic model to use (function)
        :param cost: cost function to use (function)
        :param footsteps: number of footsteps in prediction horizon (int)
        :param intervals: number of control invervals per footstep (int)
        :return:
        """
        # ------------------ state variables (calculated by the solver)
        self.N = footsteps
        # self.X = self.opti.variable(4, self.N + 1)   # state trajectory
        self.D = self.opti.variable(4, self.N + 1)   # DCM state trajectory (d_x, d_y, d_dot_x, d_dot_y)

        # ------------------ optimisation variables (optimised by the solver)
        self.U = self.opti.variable(2, self.N)      # footstep locations
        self.dT = self.opti.variable(1, self.N)     # footstep durations

        # ------------------ parameters (updated each iteration)
        self.X0 = self.opti.parameter(4)    # current state
        self.D0 = self.opti.parameter(4)    # current DCM state
        self.dt = self.opti.parameter()     # current step duration
        self.U0 = self.opti.parameter(2)    # support foot location
        self.Xs0 = self.opti.parameter(2)   # swing foot location
        self.flag = self.opti.parameter()   # support foot flag; 1 = right, -1 = left
        self.v_ref = self.opti.parameter(2) # reference velocity
        self.Q = self.opti.parameter(2, 2)  # weighting matrix

        # ------------------ system dynamics
        # print('------------------------------------')
        # print('system dyanmics', model)
        # print('--------------------------------------')
        # for i in range(self.N):
        #     if model == self.dcm_dynamics:
        #         x_current = self.X[:, i]
        #         d_current = self.D[:, i]
        #         u_current = self.U[:, i]
        #         delta_t = self.dT[i]
        #         d_next = model(d_current, u_current, delta_t, intervals)
        #         self.opti.subject_to(self.D[:, i + 1] == d_next)
        #         x_next = model(x_current, u_current, delta_t, intervals)
        #         self.opti.subject_to(self.X[:, i + 1] == x_next)
        #     else:
        #         x_current = self.X[:, i]
        #         u_current = self.U[:, i]
        #         delta_t = self.dT[i]
        #         x_next = model(x_current, u_current, delta_t, intervals)
        #         self.opti.subject_to(self.X[:, i + 1] == x_next)
        self.x_current_0 = self.X0
        u_current_0 = self.U[:, 0]
        delta_t_0 = self.dT[0]
        self.x_next_1 = model(self.x_current_0, u_current_0, delta_t_0, intervals)

        # x_current_1 = self.X[:, 1]
        u_current_1 = self.U[:, 1]
        delta_t_1 = self.dT[1]
        self.x_next_2 = model(self.x_next_1, u_current_1, delta_t_1, intervals)
        # self.con16 = self.X[:, 2] - x_next_2 == 0
        # self.opti.subject_to(self.con16)

        # x_current_2 = self.X[:, 2]
        u_current_2 = self.U[:, 2]
        delta_t_2 = self.dT[2]
        self.x_next_3 = model(self.x_next_2, u_current_2, delta_t_2, intervals)
        # self.con17 = self.X[:, 3] - x_next_3 == 0
        # self.opti.subject_to(self.con17)

        # ------------------- cost function
        self.J = cost()
        self.opti.minimize(self.J)

        if model == self.dcm_dynamics:
            self.initialise_dcm_constraints()
        else:
            self.initialise_constraints()
        self.initialise_solver()

    # def initialise_constraints(self):
    #     self.opti.subject_to(self.X[:, 0] == self.X0)
    #     self.opti.subject_to(self.D[:, 0] == self.D0)
    #     self.opti.subject_to(self.U[:, 0] == self.U0)
    #     self.opti.subject_to(self.dT[0] + self.dt >= 0.2)
    #     s_foot = self.flag
    #     for k in range(1, self.N):
    #         # Step duration limits
    #         self.opti.subject_to(self.dT[k] >= 0.4)
    #         self.opti.subject_to(self.dT[k] <= 1.0)
    #
    #         # Prevent feet from crossing
    #         self.opti.subject_to(s_foot * (self.U[1, k] - self.U[1, k - 1]) >= self.foot_r)
    #
    #         # opti.subject_to((U[0,k]-U[0, k-1])**2 + (U[1,k]-U[1, k-1])**2 >= foot_radius ** 2)
    #         #     # opti.subject_to(U[0, k+1] - U[0, k-1] <= np.sqrt(L ** 2 - h ** 2)*0.707) # strict constrain
    #         #     opti.subject_to(s_foot*(U[1, k+1] - U[1, k-1]) <= np.sqrt(L ** 2 - h ** 2)*0.707)  # strict constrain
    #         #     opti.subject_to((U[0, k+1] - U[0, k-1])**2 + (U[1, k+1] - U[1, k-1])**2 <= L ** 2 - h ** 2)
    #
    #         # don't travel too far on the CURRENT leg
    #         self.opti.subject_to((self.U[0, k] - self.X[0, k]) ** 2 +
    #                              (self.U[1, k] - self.X[1, k]) ** 2 <=
    #                              0.5*(self.L ** 2 - self.h ** 2))
    #
    #         # don't place the NEXT leg too far away
    #         self.opti.subject_to((self.X[0, k + 1] - self.U[0, k]) ** 2 +
    #                              (self.X[1, k + 1] - self.U[1, k]) ** 2 <=
    #                              0.5*(self.L ** 2 - self.h ** 2))
    #         s_foot = s_foot * (-1)

    def initialise_constraints(self):
        self.con1 = 0.2 - self.dT[0] - self.dt <= 0.0
        self.con2 = self.dT[0] + self.dt - 0.6 <= 0.0
        self.con3 = 0.4 - self.dT[1] <= 0.0
        self.con4 = self.dT[1] - 0.6 <= 0.0
        self.con5 = 0.4 - self.dT[2] <= 0.0
        self.con6 = self.dT[2] - 0.6 <= 0.0
        self.con7 = (self.U[0, 1] - self.x_next_1[0]) ** 2 + (self.U[1, 1] - self.x_next_1[1]) ** 2 - 0.6*(self.L ** 2 - self.h ** 2) <= 0.0
        self.con8 = (self.U[0, 2] - self.x_next_2[0]) ** 2 + (self.U[1, 2] - self.x_next_2[1]) ** 2 - 0.6*(self.L ** 2 - self.h ** 2) <= 0.0
        self.con9 = (self.x_next_2[0] - self.U[0, 1]) ** 2 + (self.x_next_2[1] - self.U[1, 1]) ** 2 - 0.6*(self.L ** 2 - self.h ** 2) <= 0.0
        self.con10 = (self.x_next_3[0] - self.U[0, 2]) ** 2 + (self.x_next_3[1] - self.U[1, 2]) ** 2 - 0.6*(self.L ** 2 - self.h ** 2) <= 0.0
        self.opti.subject_to(self.con1)
        self.opti.subject_to(self.con2)
        self.opti.subject_to(self.con3)
        self.opti.subject_to(self.con4)
        self.opti.subject_to(self.con5)
        self.opti.subject_to(self.con6)
        self.opti.subject_to(self.con7)
        self.opti.subject_to(self.con8)
        self.opti.subject_to(self.con9)
        self.opti.subject_to(self.con10)
        s_foot = self.flag
        s_foot_2 = s_foot * (-1)
        self.con11 = self.foot_r - s_foot * (self.U[1, 1] - self.U[1, 0]) <= 0.0
        self.con12 = self.foot_r - s_foot_2 * (self.U[1, 2] - self.U[1, 1]) <= 0.0
        self.opti.subject_to(self.con11)
        self.opti.subject_to(self.con12)
        self.con13 = self.U[:, 0] == self.U0
        self.opti.subject_to(self.con13)

    def initialise_dcm_constraints(self):
        self.opti.subject_to(self.D[:, 0] == self.D0)
        self.opti.subject_to(self.U[:, 0] == self.U0)
        self.opti.subject_to(self.dT[0] + self.dt >= 0.2)
        s_foot =  self.flag
        for k in range(1, self.N):
            # Step duration limits
            self.opti.subject_to(self.dT[k] >= 0.4)
            self.opti.subject_to(self.dT[k] <= 0.6)

            # Prevent feet from crossing
            self.opti.subject_to(s_foot * (self.U[1, k] - self.U[1, k - 1]) >= self.foot_r)

            # opti.subject_to((U[0,k]-U[0, k-1])**2 + (U[1,k]-U[1, k-1])**2 >= foot_radius ** 2)
            #     # opti.subject_to(U[0, k+1] - U[0, k-1] <= np.sqrt(L ** 2 - h ** 2)*0.707) # strict constrain
            #     opti.subject_to(s_foot*(U[1, k+1] - U[1, k-1]) <= np.sqrt(L ** 2 - h ** 2)*0.707)  # strict constrain
            #     opti.subject_to((U[0, k+1] - U[0, k-1])**2 + (U[1, k+1] - U[1, k-1])**2 <= L ** 2 - h ** 2)

            # don't travel too far on the CURRENT leg
            self.opti.subject_to((self.U[0, k] - self.D[0, k]) ** 2 +
                                 (self.U[1, k] - self.D[1, k]) ** 2 <=
                                 0.6*(self.L ** 2 - self.h ** 2))

            # don't place the NEXT leg too far away
            self.opti.subject_to((self.D[0, k + 1] - self.U[0, k]) ** 2 +
                                 (self.D[1, k + 1] - self.U[1, k]) ** 2 <=
                                 0.6*(self.L ** 2 - self.h ** 2))
            s_foot = s_foot * (-1)

    def initialise_solver(self):
        options = {"ipopt.print_level": 0, 
                    # "print_out": True, 
                    # "print_in": True, 
                    # "print_time": True,
                   "ipopt.hessian_approximation": "exact", 
                   "ipopt.max_iter": 600,
                   "ipopt.warm_start_init_point": "yes",
                   "ipopt.bound_mult_init_method": "mu-based",
                   "ipopt.mu_init": 0.005,


                   "ipopt.warm_start_slack_bound_push": 0.0001,
                   #"ipopt.warm_start_slack_bound_frac": 0.01
                   #"ipopt.warm_start_bound_push": 0.00001
                   "ipopt.warm_start_same_structure": "no"
                   }  # "verbose": True, ,





        self.opti.solver("ipopt", options)

    def set_weightings(self, Q):
        self.opti.set_value(self.Q, Q)

    def set_parameters(self, X0, D0, dt, U0, Xs0, flag, v_ref):
        self.opti.set_value(self.X0, X0)
        self.opti.set_value(self.D0, D0)
        self.opti.set_value(self.dt, dt)
        self.opti.set_value(self.U0, U0)
        self.opti.set_value(self.Xs0, Xs0)
        self.opti.set_value(self.flag, flag)
        self.opti.set_value(self.v_ref, v_ref)

    def set_initial_solution(self, dT_i, U_i):
        self.opti.set_initial(self.dT, dT_i)
        self.opti.set_initial(self.U, U_i)

    def solve(self):
        start = time.time()
        self.sol = self.opti.solve()
        self.solve_time = time.time() - start
        self.U_opt = self.sol.value(self.U)
        self.dT_opt = self.sol.value(self.dT)
        # self.X_opt = self.sol.value(self.X)
        self.J_opt = self.sol.value(self.J)
        self.W_opt = np.array([self.sol.value(self.opti.dual(self.con1)),self.sol.value(self.opti.dual(self.con2)),self.sol.value(self.opti.dual(self.con3)),
                               self.sol.value(self.opti.dual(self.con4)),self.sol.value(self.opti.dual(self.con5)),self.sol.value(self.opti.dual(self.con6)),
                               self.sol.value(self.opti.dual(self.con7)),self.sol.value(self.opti.dual(self.con8)),self.sol.value(self.opti.dual(self.con9)),
                               self.sol.value(self.opti.dual(self.con10)),self.sol.value(self.opti.dual(self.con11)),self.sol.value(self.opti.dual(self.con12))])

    def step_velocity_cost_function(self):
        error = vertcat(horzcat(self.x_current_0[2] - self.v_ref[0], self.x_next_1[2] - self.v_ref[0],self.x_next_2[2] - self.v_ref[0],self.x_next_3[2] - self.v_ref[0]),
                        horzcat(self.x_current_0[3] - self.v_ref[1], self.x_next_1[3] - self.v_ref[1],self.x_next_2[3] - self.v_ref[1],self.x_next_3[3] - self.v_ref[1]))

        J = self.Q[0, 0] * mtimes(error[0, :], error[0, :].T) + \
            2 * self.Q[0, 1] * mtimes(error[0, :], error[1, :].T) + \
            self.Q[1, 1] * mtimes(error[1, :], error[1, :].T)
        return J

    def scaled_step_velocity_cost_function(self):
        x_dots = self.X[2, 1:]
        y_dots = self.X[3, 1:]
        cum_dT = cumsum(self.dT)
        print(cum_dT)
        T = cum_dT[-1]
        scaled_v_ref_x = self.X0[2] * (T - cum_dT) / T + self.v_ref[0] * cum_dT / T
        scaled_v_ref_y = self.X0[3] * (T - cum_dT) / T + self.v_ref[1] * cum_dT / T
        error = vertcat(x_dots - scaled_v_ref_x, y_dots - scaled_v_ref_y)
        J = self.Q[0, 0] * mtimes(error[0, :], error[0, :].T) + \
            2 * self.Q[0, 1] * mtimes(error[0, :], error[1, :].T) + \
            self.Q[1, 1] * mtimes(error[1, :], error[1, :].T)
        return J

    def avg_velocity_cost_function(self):
        # dxs = self.X[0, 1:] - self.X[0, :-1]
        # dys = self.X[1, 1:] - self.X[1, :-1]
        dxs = self.X[0, 1:3] - self.X[0, 0:2]
        dys = self.X[1, 1:3] - self.X[1, 0:2]
        # avg_v_x = dxs / self.dT
        # avg_v_y = dys / self.dT
        # x_1 / dT_1 + x_2 / dT_
        # {1 + 2} + x_3 / dT_
        # {1 + 2 + 3}
        # dx1 = self.X[0, 1] - self.X[0, 0]
        # dx2 = self.X[0, 2] - self.X[0, 1]
        # dx3 = self.X[0, 3] - self.X[0, 2]
        # dy1 = self.X[1, 1] - self.X[1, 0]
        # dy2 = self.X[1, 2] - self.X[1, 1]
        # dy3 = self.X[1, 3] - self.X[1, 2]
        # dT1 = cumsum(self.dT)[0]
        # dT2 = cumsum(self.dT)[1]
        # dT3 = cumsum(self.dT)[2]
        # error = vertcat(dx1/dT1+dx2/dT2+dx3/dT3-self.v_ref[0]*3, dy1/dT1+dy2/dT2+dy3/dT3-self.v_ref[1]*3)
        # dx = self.X[0, -1] - self.X[0, 0]
        # dy = self.X[1, -1] - self.X[1, 0]
        dT = cumsum(self.dT)[:2]
        # dT = self.dT
        # dT = self.dT[0]
        # avg_v_x = dx
        # / dT
        # avg_v_y = dy / dT
        avg_v_x = dxs/dT
        avg_v_y = dys/dT
        error = vertcat(avg_v_x - self.v_ref[0], avg_v_y - self.v_ref[1])
        J = self.Q[0, 0] * mtimes(error[0, :], error[0, :].T) + \
            2 * self.Q[0, 1] * mtimes(error[0, :], error[1, :].T) + \
            self.Q[1, 1] * mtimes(error[1, :], error[1, :].T)
        return J

    def avg_dcm_velocity_cost_function(self):
        d_dx = self.D[0, -1] - self.D[0, 0]
        d_dy = self.D[1, -1] - self.D[1, 0]
        dT = cumsum(self.dT)[-1]
        avg_dcm_v_x = d_dx / dT
        avg_dcm_v_y = d_dy / dT
        error = vertcat(avg_dcm_v_x - self.v_ref[0], avg_dcm_v_y - self.v_ref[1])
        J = self.Q[0, 0] * mtimes(error[0, :], error[0, :].T) + \
            2 * self.Q[0, 1] * mtimes(error[0, :], error[1, :].T) + \
            self.Q[1, 1] * mtimes(error[1, :], error[1, :].T)
        return J

    def full_dynamics(self, x_current, u_current, delta_t, intervals):
        a_x = 0.5 * (x_current[0] - u_current[0] + x_current[2] / self.omega)
        a_y = 0.5 * (x_current[1] - u_current[1] + x_current[3] / self.omega)
        b_x = 0.5 * (x_current[0] - u_current[0] - x_current[2] / self.omega)
        b_y = 0.5 * (x_current[1] - u_current[1] - x_current[3] / self.omega)
        # x_1 = u_current[0] + (x_current[0] - u_current[0]) * cosh(self.omega * delta_t) + (x_current[2] / self.omega) * sinh(self.omega * delta_t)
        # y_1 = u_current[1] + (x_current[1] - u_current[1]) * cosh(self.omega * delta_t) + (x_current[3] / self.omega) * sinh(self.omega * delta_t)
        # x_dot_1 = self.omega * (x_current[0] - u_current[0]) * sinh(self.omega * delta_t) + x_current[2] * cosh(self.omega * delta_t)
        # y_dot_1 = self.omega * (x_current[1] - u_current[1]) * sinh(self.omega * delta_t) + x_current[3] * cosh(self.omega * delta_t)
        x_1 = a_x * exp(self.omega * delta_t) + b_x * exp(- self.omega * delta_t) + u_current[0]
        y_1 = a_y * exp(self.omega * delta_t) + b_y * exp(- self.omega * delta_t) + u_current[1]
        x_dot_1 = a_x * self.omega * exp(self.omega * delta_t) - b_x * self.omega * exp(- self.omega * delta_t)
        y_dot_1 = a_y * self.omega * exp(self.omega * delta_t) - b_y * self.omega * exp(- self.omega * delta_t)
        x_next = vertcat(x_1, y_1, x_dot_1, y_dot_1)
        return x_next

    def rk4_dynamics(self, x_current, u_current, delta_t, intervals):
        f = lambda x, u: vertcat(x[2], x[3], (x[0] - u[0]) * g / self.h,
                                 (x[1] - u[1]) * g / self.h)  # dx/dt = f(x,u)
        x_next = x_current
        dt_ = delta_t / intervals
        for i in range(intervals):
            # Runge-Kutta 4 integration
            k1 = f(x_current, u_current)
            k2 = f(x_current + dt_ / 2 * k1, u_current)
            k3 = f(x_current + dt_ / 2 * k2, u_current)
            k4 = f(x_current + dt_ * k3, u_current)
            x_next = x_current + dt_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            x_current = x_next
        return x_next

    def dcm_dynamics(self, d_current, u_current, delta_t, intervals):
        d_x_0 = d_current[0]
        d_y_0 = d_current[1]
        u_x = u_current[0]
        u_y = u_current[1]

        d_x_1 = d_x_0 * exp(self.omega * delta_t) + u_x * (1 - exp(self.omega * delta_t))
        d_y_1 = d_y_0 * exp(self.omega * delta_t) + u_y * (1 - exp(self.omega * delta_t))
        d_dot_x_1 = self.omega * (d_x_0 - u_x) * exp(self.omega * delta_t)
        d_dot_y_1 = self.omega * (d_y_0 - u_y) * exp(self.omega * delta_t)
        d_next = vertcat(d_x_1, d_y_1, d_dot_x_1, d_dot_y_1)
        return d_next


def full_dynamic_model(X_k, U_k, dt):
    m = 15
    L = 1.2
    h = 0.7

    x_k = X_k[0]
    y_k = X_k[1]
    x_dot_k = X_k[2]
    y_dot_k = X_k[3]

    u_x_k = U_k[0]
    u_y_k = U_k[1]

    omega = np.sqrt(g/h)

    x_k_1 = u_x_k + (x_k - u_x_k) * np.cosh(omega * dt) + (x_dot_k / omega) * np.sinh(omega * dt)
    y_k_1 = u_y_k + (y_k - u_y_k) * np.cosh(omega * dt) + (y_dot_k / omega) * np.sinh(omega * dt)
    x_dot_k_1 = x_dot_k * np.cosh(omega * dt) + (x_k - u_x_k) * omega * np.sinh(omega * dt)
    y_dot_k_1 = y_dot_k * np.cosh(omega * dt) + (y_k - u_y_k) * omega * np.sinh(omega * dt)
    X_k_1 = np.array([x_k_1, y_k_1, x_dot_k_1, y_dot_k_1])
    return X_k_1


