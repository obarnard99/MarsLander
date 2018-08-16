# imports
import numpy as np
import matplotlib.pyplot as plt

# mass of Mars and G
mM = 6.42e23
G = 6.674e-11

def Euler(s_init,v_init,t,min_radius=3390000):
    # initialise lists to record trajectories
    s = np.array([s_init])
    v = np.array([v_init])

    # determine dt
    dt = t[1] - t[0]

    # Euler integration
    for t in t[1:]:
        # calculate new position and velocity
        a = -G * mM * s[-1] / (np.linalg.norm(s[-1]) ** 3)
        new_s = s[-1] + dt * v[-1]
        new_v = v[-1] + dt * a

        # append current state to trajectories
        s = np.append(s,[new_s],axis=0)
        v = np.append(v,[new_v],axis=0)

        # break out of loop when lander reaches surface of Mars
        if np.linalg.norm(s[-1]) <= min_radius:
            return s, v

    return s, v

def Verlet(s_init,v_init,t,min_radius=3390000):
    # initialise lists to record trajectories
    s = np.array([s_init])
    v = np.array([v_init])

    # determine dt
    dt = t[1] - t[0]

    # Use Euler to get second value for x ???????????????????
    new_s = s[0] + dt * v[0]
    s = np.append(s, [new_s], axis=0)

    # Verlet integration
    for t in t[2:]:
        # calculate new position and velocity
        a = -G * mM * s[-1] / (np.linalg.norm(s[-1]) ** 3)
        new_s = 2 * s[-1] - s[-2] + (dt ** 2) * a
        s = np.append(s, [new_s], axis=0)

        new_v = (s[-1] - s[-3]) / (2 * dt)
        v = np.append(v, [new_v], axis=0)

        # break out of loop when lander reaches surface of Mars
        if np.linalg.norm(s[-1]) <= min_radius:
            # append velocity array with final velocity
            new_v = (s[-1] - s[-2]) / dt
            v = np.append(v, [new_v], axis=0)
            return s, v

    # append velocity array with final velocity
    new_v = (s[-1] - s[-2]) / dt
    v = np.append(v, [new_v], axis=0)
    return s, v