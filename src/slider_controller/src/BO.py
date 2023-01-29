from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64

import matplotlib.pyplot as plt
from matplotlib import gridspec

u_s_rk4 = np.zeros((2,2*500))
u_s_gd = np.zeros((2,2*500))
t_s_rk4 = np.zeros((3*500))
t_s_gd = np.zeros((3*500))
count = 0
rk4_flag = False
gd_flag = False
gradients = np.zeros(7*30)

w_result = np.zeros((6, 500))
state_result = np.zeros((10, 500))

# def square_error(u_s_err, t_s_err):
#     return -(np.sum(u_s_err.flatten()**2) + np.sum(t_s_err**2))

def square_error(gd_err):
    return -np.sqrt(np.sum(gd_err**2))/30

def posterior(optimizer, x_obs, y_obs, grid):
    optimizer._gp.fit(x_obs, y_obs)
    mu, sigma = optimizer._gp.predict(grid, return_std=True)
    return mu, sigma

def plot_gp(optimizer, x, state, xlim):
    fig = plt.figure(figsize=(16, 10))
    print('Figure view')
    steps = len(optimizer.space)
    fig.suptitle(
        'Gaussian Process and Utility Function After {} Steps'.format(steps),
        fontdict={'size':30}
    )
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1]) 
    axis = plt.subplot(gs[0])
    # acq = plt.subplot(gs[1])
    
    # x_obs = np.array([[res["params"][state]] for res in optimizer.res])
    # y_obs = np.array([res["target"] for res in optimizer.res])
    x_obs = []
    y_obs = []
    for res in optimizer.res:
        sum = 0
        for i in range(1,7):
            sum += res["params"]["w"+str(i)]
        x_obs.append(sum)
        y_obs.append(res["target"])

    print('The x_obs is ', x_obs)
    print('The y_obs is ', y_obs)
    x_obs = np.reshape(x_obs, (len(x_obs), 1))
    y_obs = np.array(y_obs)
    np.save('result_1.npy', x_obs, y_obs)
    # y_obs = np.reshape(y_obs, (len(y_obs), 1))

    mu, sigma = posterior(optimizer, x_obs, y_obs, x)
    # axis.plot(x, y, linewidth=3, label='Target')
    axis.plot(x_obs.flatten(), y_obs, 'D', markersize=8, label='Observations', color='r')
    axis.plot(x, mu, '--', color='k', label='Prediction')
    yy = -6.5427*np.ones((100,1))
    axis.plot(x, yy, '--', color='r')

    axis.fill(np.concatenate([x, x[::-1]]), 
              np.concatenate([mu - 1.9600 * sigma, (mu + 1.9600 * sigma)[::-1]]),
        alpha=.6, fc='c', ec='None', label='95% confidence interval')
    
    axis.set_xlim((-1, xlim))
    axis.set_ylim((min(y_obs)*1.5, -min(y_obs)))
    axis.set_ylabel('f(x)', fontdict={'size':20})
    axis.set_xlabel('x', fontdict={'size':20})
    
    # utility_function = UtilityFunction(kind="ucb", kappa=3, xi=0)
    # utility = utility_function.utility(x, optimizer._gp, 0)
    # acq.plot(x, utility, label='Utility Function', color='purple')
    # acq.plot(x[np.argmax(utility)], np.max(utility), '*', markersize=15, 
    #          label=u'Next Best Guess', markerfacecolor='gold', markeredgecolor='k', markeredgewidth=1)
    # acq.set_xlim((0, xlim))
    # acq.set_ylim((0, np.max(utility) + 0.5))
    # acq.set_ylabel('Utility', fontdict={'size':20})
    # acq.set_xlabel('x', fontdict={'size':20})
    
    axis.legend(loc=2, bbox_to_anchor=(0.7, 0.95), borderaxespad=0.)
    # acq.legend(loc=2, bbox_to_anchor=(1.01, 1), borderaxespad=0.)

    plt.show()

def planner_rk4_callback(msg):
    global u_s_rk4, t_s_rk4, rk4_flag, count
    u_s_rk4[0,0+count*2] = msg.data[0]
    u_s_rk4[1,0+count*2] = msg.data[1]
    u_s_rk4[0,1+count*2] = msg.data[2]
    u_s_rk4[1,1+count*2] = msg.data[3]
    t_s_rk4[0+count*3] = msg.data[4]
    t_s_rk4[1+count*3] = msg.data[5]
    t_s_rk4[2+count*3] = msg.data[6]
    rk4_flag = True
    # count += 1
    # print('The received RK4 data is ')
    # print('The received RK4 data is ', u_s_rk4)

def planner_callback(msg):
    global u_s_gd, t_s_gd, gd_flag, count
    u_s_gd[0,0+count*2] = msg.data[0]
    u_s_gd[1,0+count*2] = msg.data[1]
    u_s_gd[0,1+count*2] = msg.data[2]
    u_s_gd[1,1+count*2] = msg.data[3]
    t_s_gd[0+count*3] = msg.data[4]
    t_s_gd[1+count*3] = msg.data[5]
    t_s_gd[2+count*3] = msg.data[6]
    # print('The received gd data is ')



def gradient_callback(msg):
    global gradients, count, gd_flag
    gradients[0+count*7] = msg.data[0]
    gradients[1+count*7] = msg.data[1]
    gradients[2+count*7] = msg.data[2]
    gradients[3+count*7] = msg.data[3]
    gradients[4+count*7] = msg.data[4]
    gradients[5+count*7] = msg.data[5]
    gradients[6+count*7] = msg.data[6]
    gd_flag = True
    # print('received gd data \n', msg.data)
    # print('The count inside callback \n')
    # print('--------------------------')
    # print('received gd data ')

if __name__== "__main__":
    rospy.init_node('BO', anonymous=True)
    pub_state = rospy.Publisher('/time_slider_gazebo/planner_input', Float64MultiArray, queue_size=1)
    pub_weight = rospy.Publisher('/time_slider_gazebo/cost_weight', Float64MultiArray, queue_size=1)
    pub_result = rospy.Publisher('/time_slider_gazebo/BO_result', Float64MultiArray, queue_size=1)
    planner_input_rk4 = rospy.Subscriber('/time_slider_gazebo/footstep_plan_rk4', Float64MultiArray, planner_rk4_callback)
    planner_input = rospy.Subscriber('/time_slider_gazebo/footstep_plan', Float64MultiArray, planner_callback)
    gd_gradient = rospy.Subscriber('/time_slider_gazebo/gd_gradients', Float64MultiArray, gradient_callback)
    lmax = 1.0
    lmin = 0.2
    r = rospy.Rate(25)

    optimizer = BayesianOptimization(
    f=None,
    pbounds={'w1':(0.01,50), 'w2':(0.01,50), 'w3':(0.01,50), 'w4':(0.01,50), 'w5':(0.01,50), 'w6':(0.01,50)},
    verbose=2,
    random_state=1,)
    optimizer.set_gp_params(normalize_y=True)
    utility = UtilityFunction(kind="ucb", kappa=10.0, xi=0.0)

    # w_data = np.zeros(6) * 1
    w_data = np.array([10.813910100830805, 9.043644014320767, 10.13765747355422, 7.032150242601438, 0.01, 2.0486204804374815])
    w = Float64MultiArray()
    BO_result = Float64MultiArray()

    start_count = 0

    x = (np.random.rand(1) * 2 - 1) * 1
    x_dot = (np.random.rand(1) * 2 - 1) * 3
    x_dotdot = (np.random.rand(1) * 2 - 1) * 3
    y = (np.random.rand(1) * 2 - 1) * 0.5
    y_dot = (np.random.rand(1) * 2 - 1) * 3
    y_dotdot = (np.random.rand(1) * 2 - 1) * 3
    seed = np.random.choice([0, 1])
    if seed == 0:
        foot_flag = -1  # left support
    else:
        foot_flag = 1

    x_left = x + np.random.rand(1) * lmax * 0.25
    y_left = y + np.random.rand(1) * lmax * 0.25
    x_right = x + np.random.rand(1) * lmax * 0.25
    y_right = y - np.random.rand(1) * lmax * 0.25
    current_time = np.random.rand(1) * 0.3
    state_random = Float64MultiArray()
    state_random.data = [x, x_dot, x_dotdot, y, y_dot, y_dotdot, x_left, y_left, x_right, y_right, foot_flag,
                         current_time]
    print('The random state is ', state_random.data)
    sample_count = 0

    while not rospy.is_shutdown():
        # x, x_dot, x_dotdot, y, y_dot, y_dotdot, left_x, left_y, right_x, right_y, foot_flag, current_time

        pub_state.publish(state_random)

        w.data = w_data
        pub_weight.publish(w)

        if rk4_flag == True and gd_flag == True:
            print("count \t", count)
            print(square_error(gradients[count*7:(count+1)*7]))
            print('Gradients are ', gradients[count*7:(count+1)*7])
            count += 1
            rk4_flag = False
            gd_flag = False
            # BO_result.data = [square_error(gradients[count*7:(count+1)*7])]
            # pub_result.publish(BO_result)

        if count == 3:
            target = square_error(gradients)
            print('target is ', target)
            # register new data to BO
            optimizer.register(params=w_data, target=target)
            # you have to provide some random samples at first
            if start_count < 5:
                w_data = np.random.random_sample((6,)) * 5
            else:
                next_point_to_probe = optimizer.suggest(utility)
                w_data = [next_point_to_probe['w1'], next_point_to_probe['w2'], next_point_to_probe['w3'],
                          next_point_to_probe['w4'], next_point_to_probe['w5'],
                          next_point_to_probe['w6']]
            start_count += 1
            count = 0
            # print('The weight is ', next_point_to_probe)
            print('The maximum is ', optimizer.max)
            print('iteration count is ', start_count)
            if start_count > 0 and start_count % 150 == 0:
                print('random state is ', state_random.data)
                w_result[:, sample_count] = np.array([optimizer.max["params"]["w1"],optimizer.max["params"]["w2"],optimizer.max["params"]["w3"],optimizer.max["params"]["w4"],optimizer.max["params"]["w5"],optimizer.max["params"]["w6"] ])
                state_result[:, sample_count] = np.array([x, x_dot, y, y_dot, foot_flag, x_left, y_left, x_right, y_right, current_time])
                x_left = x + np.random.rand(1) * lmax * 0.25
                y_left = y + np.random.rand(1) * lmax * 0.25
                x_right = x + np.random.rand(1) * lmax * 0.25
                y_right = y - np.random.rand(1) * lmax * 0.25
                current_time = np.random.rand(1) * 0.3
                state_random = Float64MultiArray()
                state_random.data = [x, x_dot, x_dotdot, y, y_dot, y_dotdot, x_left, y_left, x_right, y_right,
                                     foot_flag,
                                     current_time]
                w_data = np.array([10.813910100830805, 9.043644014320767, 10.13765747355422, 7.032150242601438, 0.01,
                                   2.0486204804374815])

                optimizer = BayesianOptimization(
                    f=None,
                    pbounds={'w1': (0.01, 50), 'w2': (0.01, 50), 'w3': (0.01, 50), 'w4': (0.01, 50), 'w5': (0.01, 50),
                             'w6': (0.01, 50)},
                    verbose=2,
                    random_state=1, )
                optimizer.set_gp_params(normalize_y=True)
                utility = UtilityFunction(kind="ucb", kappa=10.0, xi=0.0)

                sample_count += 1
                start_count = 0
                if sample_count >= 500:
                    print('sampling done!')
                    np.savez('result.npz', weight = w_result, state = state_result)
                    break

                # xa = np.linspace(0, 30, 100).reshape(-1, 1)
                # plot_gp(optimizer, xa, "w1", 30)

        # print('The count is ', count)
        r.sleep()

    #
    # next_point = {'y0': 11}
    # target = -5.7789
    # optimizer.register(params = next_point, target = target)
    # # 6.5427
    # xa = np.linspace(-1, 15, 200).reshape(-1, 1)
    # plot_gp(optimizer,xa, "y0", 15)



