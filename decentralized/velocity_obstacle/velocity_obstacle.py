"""
Collision avoidance using Velocity-obstacle method

author: Ashwin Bose (atb033@github.com)
"""

from utils.multi_robot_plot import plot_robot_and_obstacles
#from utils.create_obstacles import create_obstacles
from utils.control import compute_desired_velocity
import numpy as np

SIM_TIME = 18.#5
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 20#0.5
VMAX = 50 #2
VMIN = 5 #0.2

# def calculate_path_length(robot_state_history):
#     total_length = 0.0
#     for i in range(1, len(robot_state_history[0])):
#         position = robot_state_history[:2, i]
#         prev_position = robot_state_history[:2, i - 1]
#         step_distance = np.linalg.norm(position - prev_position)
#         total_length += step_distance
#     return total_length


def simulate(filename):
    #obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([0, 300, 0, 0])
    start2 = np.array([450, 600, 0, 0])
    start3 = np.array([900, 300, 0, 0])
    start4 = np.array([450, 0, 0, 0])

    goal = np.array([900, 300, 0, 0])
    goal2 = np.array([450, 0, 0, 0])
    goal3 = np.array([0, 300, 0, 0])
    goal4 = np.array([450, 600, 0, 0])

    robot_state = start
    robot_state2 = start2
    robot_state3 = start3
    robot_state4 = start4
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))
    robot_state_history2 = np.empty((4, NUMBER_OF_TIMESTEPS))
    robot_state_history3 = np.empty((4, NUMBER_OF_TIMESTEPS))
    robot_state_history4 = np.empty((4, NUMBER_OF_TIMESTEPS))

    for i in range(NUMBER_OF_TIMESTEPS):
        v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX)
        v_desired2 = compute_desired_velocity(robot_state2, goal2, ROBOT_RADIUS, VMAX)
        v_desired3 = compute_desired_velocity(robot_state3, goal3, ROBOT_RADIUS, VMAX)
        v_desired4 = compute_desired_velocity(robot_state4, goal4, ROBOT_RADIUS, VMAX)
        
        control_vel = compute_velocity(
            robot_state, robot_state2, robot_state3, robot_state4, v_desired)
  
        control_vel2 = compute_velocity(
            robot_state2, robot_state, robot_state3, robot_state4, v_desired2)
      
        control_vel3 = compute_velocity(
            robot_state3, robot_state, robot_state2, robot_state4, v_desired3)
        
        control_vel4 = compute_velocity(
            robot_state4, robot_state2, robot_state3, robot_state, v_desired4)

        robot_state = update_state(robot_state, control_vel)
        robot_state2 = update_state(robot_state2, control_vel2)
        robot_state3 = update_state(robot_state3, control_vel3)
        robot_state4 = update_state(robot_state4, control_vel4)

        robot_state_history[:4, i] = robot_state
        robot_state_history2[:4, i] = robot_state2
        robot_state_history3[:4, i] = robot_state3
        robot_state_history4[:4, i] = robot_state4

    plot_robot_and_obstacles(
        robot_state_history, robot_state_history2, robot_state_history3, robot_state_history4, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)
    
    # robot1_path_length = calculate_path_length(robot_state_history)
    # robot2_path_length = calculate_path_length(robot_state_history2)
    # robot3_path_length = calculate_path_length(robot_state_history3)
    # robot4_path_length = calculate_path_length(robot_state_history4)


def compute_velocity(robot, sub_robot1, sub_robot2, sub_robot3, v_desired):
    pA = robot[:2]
    vA = robot[-2:]
    pA2 = sub_robot1[:2]
    vA2 = sub_robot1[-2:]
    pA3 = sub_robot2[:2]
    vA3 = sub_robot2[-2:]
    pA4 = sub_robot3[:2]
    vA4 = sub_robot3[-2:]
    
    # Compute the constraints
    # for each velocity obstacles
    # number_of_obstacles = np.shape(obstacles)[1]
    number_of_obstacles = 4
    Amat = np.empty((4 * 2, 2))
    bvec = np.empty((4 * 2))
    Amat2 = np.empty((4 * 2, 2))
    bvec2 = np.empty((4 * 2))
    Amat3 = np.empty((4 * 2, 2))
    bvec3 = np.empty((4 * 2))
    #
    for i in range(number_of_obstacles):
        #obstacle = obstacles[:, i]
        dispA1_2 = pA - pA2
        dispA1_3 = pA - pA3
        dispA1_4 = pA - pA4

        distBA = np.linalg.norm(dispA1_2)
        distBA2 = np.linalg.norm(dispA1_3)
        distBA3 = np.linalg.norm(dispA1_4)

        thetaBA = np.arctan2(dispA1_2[1], dispA1_2[0])
        thetaBA2 = np.arctan2(dispA1_3[1], dispA1_3[0])
        thetaBA3 = np.arctan2(dispA1_4[1], dispA1_4[0])

        if 2.2 * ROBOT_RADIUS > distBA:
            distBA = 2.2*ROBOT_RADIUS
        phi_obst = np.arcsin(2.2*ROBOT_RADIUS/distBA)
        phi_left = thetaBA + phi_obst
        phi_right = thetaBA - phi_obst

        if 2.2 * ROBOT_RADIUS > distBA2:
            distBA2 = 2.2*ROBOT_RADIUS
        phi_obst2 = np.arcsin(2.2*ROBOT_RADIUS/distBA2)
        phi_left2 = thetaBA2 + phi_obst2
        phi_right2 = thetaBA2 - phi_obst2

        if 2.2 * ROBOT_RADIUS > distBA3:
            distBA3 = 2.2*ROBOT_RADIUS
        phi_obst3 = np.arcsin(2.2*ROBOT_RADIUS/distBA3)
        phi_left3 = thetaBA3 + phi_obst3
        phi_right3 = thetaBA3 - phi_obst3

        # VO
        translation = vA2
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp

        translation2 = vA3
        Atemp2, btemp2 = create_constraints(translation2, phi_left2, "left")
        Amat2[i*2, :] = Atemp2
        bvec2[i*2] = btemp2
        Atemp2, btemp2 = create_constraints(translation2, phi_right2, "right")
        Amat2[i*2 + 1, :] = Atemp2
        bvec2[i*2 + 1] = btemp2

        translation3 = vA4
        Atemp3, btemp3 = create_constraints(translation3, phi_left3, "left")
        Amat3[i*2, :] = Atemp3
        bvec3[i*2] = btemp3
        Atemp3, btemp3 = create_constraints(translation3, phi_right3, "right")
        Amat3[i*2 + 1, :] = Atemp3
        bvec3[i*2 + 1] = btemp3

    # Create search-space
    th = np.linspace(0, 2*np.pi, 20)
    vel = np.linspace(0, VMAX, 5)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)
    # v_satisfying_constraints2 = check_constraints(v_sample, Amat2, bvec2)
    # v_satisfying_constraints3 = check_constraints(v_sample, Amat3, bvec3)

    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])

    # size = np.shape(v_satisfying_constraints2)[1]
    # diffs = v_satisfying_constraints - \
    #     ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    # norm = np.linalg.norm(diffs, axis=0)
    # min_index = np.where(norm == np.amin(norm))[0][0]
    # cmd_vel2 = (v_satisfying_constraints[:, min_index])

    # size = np.shape(v_satisfying_constraints3)[1]
    # diffs = v_satisfying_constraints - \
    #     ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    # norm = np.linalg.norm(diffs, axis=0)
    # min_index = np.where(norm == np.amin(norm))[0][0]
    # cmd_vel3 = (v_satisfying_constraints[:, min_index])


    return cmd_vel


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]

    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample


def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] < bvec).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == "left":
        line *= -1

    A = line[:2]
    b = -line[2]

    return A, b


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return matrix @ line


def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state
