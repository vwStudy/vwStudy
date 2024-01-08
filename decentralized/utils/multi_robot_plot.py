"""
Plotting tool for 2D multi-robot system

author: Ashwin Bose (@atb033)
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np

def plot_robot_and_obstacles(robot, robot2, robot3, robot4, robot_radius, num_steps, sim_time, filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(150, 700), ylim=(100, 400))
    ax.set_aspect('equal')
    ax.grid()
    line, = ax.plot([], [], '--r')
    line2, = ax.plot([], [], '--r')
    line3, = ax.plot([], [], '--r')
    line4, = ax.plot([], [], '--r')

    robot_patch = Circle((robot[0, 0], robot[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    
    robot_patch2 = Circle((robot2[0, 0], robot2[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    
    robot_patch3 = Circle((robot3[0, 0], robot3[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    
    robot_patch4 = Circle((robot4[0, 0], robot4[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    def init():
        ax.add_patch(robot_patch)
        ax.add_patch(robot_patch2)
        ax.add_patch(robot_patch3)
        ax.add_patch(robot_patch4)

        # for obstacle in obstacle_list:
        #     ax.add_patch(obstacle)
        line.set_data([], [])
        line2.set_data([], [])
        line3.set_data([], [])
        line4.set_data([], [])

        return [robot_patch] + [robot_patch2] + [robot_patch3] + [robot_patch4] + [line] + [line2] + [line3] + [line4]

    def animate(i):
        robot_patch.center = (robot[0, i], robot[1, i])
        robot_patch2.center = (robot2[0, i], robot2[1, i])
        robot_patch3.center = (robot3[0, i], robot3[1, i])
        robot_patch4.center = (robot4[0, i], robot4[1, i])

        # for j in range(len(obstacle_list)):
        #     obstacle_list[j].center = (obstacles[0, i, j], obstacles[1, i, j])
        line.set_data(robot[0, :i], robot[1, :i])
        line2.set_data(robot2[0, :i], robot2[1, :i])
        line3.set_data(robot3[0, :i], robot3[1, :i])
        line4.set_data(robot4[0, :i], robot4[1, :i])
        return [robot_patch] + [robot_patch2] + [robot_patch3] + [robot_patch4] + [line] + [line2] + [line3] + [line4]

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)

    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=200,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)


def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    # if is_obstacle:
    #     circle = plt.Circle((x, y), radius, color='aqua', ec='black')
    #     plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)
    # else:
    circle = plt.Circle((x, y), radius, color='green', ec='black')
    plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')

    plt.gcf().gca().add_artist(circle)
