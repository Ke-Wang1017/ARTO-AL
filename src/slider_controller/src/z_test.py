from z_planner import SLIP
import matplotlib.pyplot as plt
# from __future__ import division
if __name__ == '__main__':
    Zc = 0.7
    step_time = 0.7
    z = []
    z_dot = []
    z_dotdot = []
    Z = SLIP(Zc, step_time)
    Z.update_state(0.7, 0.0, 0.8, 0.85)
    for i in range(100):
        i = i/1.0
        zz, zz_dot, zz_dotdot = Z.get_com_state(i/100*0.7)
        z.append(zz)
        z_dot.append(zz_dot)
        z_dotdot.append(zz_dotdot)
    print('The final r_t is ', Z.r(0.7))
    print('The final z is ', zz)

    plt.plot(z)
    plt.show()
    plt.plot(z_dot)
    plt.show()
    plt.plot(z_dotdot)
    plt.show()
