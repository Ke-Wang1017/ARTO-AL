import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 22})


def offset_radial_axis(ax):
    CIRCLE_RES = 360  # resolution of circle inside
    x_circle = np.linspace(0, 2*np.pi, CIRCLE_RES)
    y_circle = np.zeros_like(x_circle)
    ax.fill(x_circle, y_circle, fc='white', ec='black', zorder=2.0) # circle
    ax.set_rmin(-0.0)
    ax.set_rticks([tick for tick in ax.get_yticks() if tick >= 0])
    # or set the ticks manually (simple)
    # or define a custom TickLocator (very flexible)
    # or leave out this line if the ticks are fully behind the circle


def add_scale(ax):
    X_OFFSET = -0.05  # to control how far the scale is from the plot (axes coordinates)
    # add extra axes for the scale
    rect = ax.get_position()
    rect = (rect.xmin-X_OFFSET, rect.ymin+rect.height/2, # x, y
            rect.width, rect.height/2) # width, height
    scale_ax = ax.figure.add_axes(rect)
    # hide most elements of the new axes
    for loc in ['right', 'top', 'bottom']:
        scale_ax.spines[loc].set_visible(False)
    scale_ax.tick_params(bottom=False, labelbottom=False)
    scale_ax.patch.set_visible(False) # hide white background
    # adjust the scale
    scale_ax.spines['left'].set_bounds(*ax.get_ylim())
    # scale_ax.spines['left'].set_bounds(0, ax.get_rmax()) # mpl < 2.2.3
    scale_ax.set_yticks(ax.get_yticks())
    scale_ax.set_ylim(ax.get_rorigin(), ax.get_rmax())
    # scale_ax.set_ylabel('$\Delta\dot{x}_{ref}$ - $ms^{-1}$')
    scale_ax.set_ylabel('Push Force - $N$')
    # scale_ax.set_ylim(ax.get_ylim()) # Matplotlib < 2.2.3


def realign_polar_xticks(ax):
    for theta, label in zip(ax.get_xticks(), ax.get_xticklabels()):
        theta = theta * ax.get_theta_direction() + ax.get_theta_offset()
        theta = np.pi/2 - theta
        y, x = np.cos(theta), np.sin(theta)
        if x >= 0.1:
            label.set_horizontalalignment('left')
        if x <= -0.1:
            label.set_horizontalalignment('right')
        if y >= 0.5:
            label.set_verticalalignment('bottom')
        if y <= -0.5:
            label.set_verticalalignment('top')


thetas = np.deg2rad(np.array([-180, -135, -90, -45, 0, 45, 90, 135, 180]))
# rk4 = [0.7, 0.65, 0.6, 0.5, 0.6, 0.65, 0.55, 0.5, 0.7]
# arto_LM = [0.85, 0.7, 0.55, 0.65, 0.75, 0.65, 0.65, 0.7, 0.85]
# arto_AL = [0.9, 0.75, 0.70, 0.7, 0.95, 0.8, 0.70, 0.75, 0.9]
nta =[30, 15, 30, 30, 30, 15, 10, 15, 30]
rk4 = [90, 45, 35, 80, 95, 25, 25, 25, 90]
arto_LM = [85, 50, 30, 50, 85, 25, 15, 20, 85]
arto_AL = [110, 70, 60, 95, 120, 40, 25, 25, 110]
ax = plt.subplot(111, projection='polar')
ax.set_theta_zero_location("N")
ax.set_rlabel_position(-90)
ax.set_rticks([0.0, 50, 100, 150, 200, 250])  # Less radial ticks
thetaticks = np.linspace(0,  360, 8, endpoint=False)

ax = plt.subplot(111, projection='polar')
ax.plot(thetas, nta, linewidth=3.0, color=[0, 0.95, 0])
ax.plot(thetas, rk4, linewidth=3.0, color=[1, 0.55, 0])
ax.plot(thetas, arto_LM, linewidth=3.0, color=[0.8, 0, 0])
ax.plot(thetas, arto_AL, linewidth=3.0, color=[0, 0, 1])
ax.grid(True)

# offset_radial_axis(ax)
realign_polar_xticks(ax)

# ax.legend(['rk4', 'RK4', 'RK4-GD'], bbox_to_anchor=(0.5, 1.17), loc="upper center", ncol=3)
ax.legend(['No-Time-Adp',  'IPOPT','ARTO-LM', 'ARTO-AL'], bbox_to_anchor=(0, 0), loc="lower right")
CIRCLE_RES = 720  # resolution of circle inside

ax.set_rticks([tick for tick in ax.get_yticks() if tick >= 50])
# ax.set_rorigin(-0.3)
x_circle = np.linspace(-np.pi, 2*np.pi, CIRCLE_RES)
y_circle = np.ones_like(x_circle) * 50
# ax.set_rmin(-0.3)
ax.fill_between(x_circle, y_circle, fc='white', ec='black', zorder=2.0) # circle
add_scale(ax)

plt.show()

# ax.set_thetagrids(thetaticks, labels=["$0^\circ$", "$45^\circ$", "$90^\circ$", "$135^\circ$", "$180^\circ$", "$225^\circ$", "", "$315^\circ$"])
#
# ax.grid(True)
# label_position=ax.get_rlabel_position()
# ax.text(np.radians(label_position-3.5), ax.get_rmax()*0.9,
#         '$\Delta\dot{x}_{ref}$ - $ms^{-1}$',
#         rotation=0, ha='center', va='center')
# ax.text(np.radians(label_position+45), ax.get_rmax()*1.0,
#         'Angle from Forwards',
#         rotation=-45, ha='center', va='center')
# ax.set_theta_label("Transverse Angle Clockwise from Forwards - degrees")
