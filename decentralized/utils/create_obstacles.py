import numpy as np


# car1_STARTtoGOAL : float = [[257.0, 250.0],[642.0, 250.0]]##左車
# car2_STARTtoGOAL : float = [[450.0, 147.0],[450.0, 363.0]]##上車 
# car3_STARTtoGOAL : float = [[642.0, 250.0],[257.0, 250.0]]##右車
# car4_STARTtoGOAL : float = [[450.0, 363.0],[450.0, 147.0]]


def create_obstacles(sim_time, num_timesteps):
    # Obstacle 1
    v = -30
    
    #最悪ケースp0 = np.array([450.0, 363.0])
    p0 = np.array([462.0, 363.0])
    obst = create_robot(p0, v, np.pi/2, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = obst
    
    # Obstacle 2
    v = 30

    # 最悪ケースp0 = np.array([257.0, 250.0])
    p0 = np.array([257.0, 250.0])
    obst = create_robot(p0, v, 0, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    
    # Obstacle 3
    v = 30
    
    #最悪ケースp0 = np.array([642.0, 250.0]) 
    p0 = np.array([642.0, 262.0])
    obst = create_robot(p0, v, np.pi, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))
    # Obstacle 4
    # v = 2
    # p0 = np.array([7.5, 2.5])
    # obst = create_robot(p0, v, np.pi * 3 / 4, sim_time, num_timesteps).reshape(4,
    #                                                                            num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))

    return obstacles


def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p
