import os
import numpy as np
from motion_primitives import MotionPrimitives
import matplotlib.pyplot as plt
from seaborn import despine

file_name = 'mprims.pickle'
if os.path.exists(file_name):
    m = MotionPrimitives(file_name)
    print('Read motion primitives from file {}'.format(file_name))
else:
    m = MotionPrimitives()

    # Define the initial states and desired goal states for the motion
    # primitives
    theta_init = np.array([0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4])

    x_vec = 0.4*np.array([3, 3, 2, 3, 3, 3, 3, 3, 1, 3, 3, 3, 3, 3, 2, 3, 3])
    y_vec = 0.4*np.array([3, 2, 2, 2, 1, 1, 1, 1, 0, -1, -1, -1, -1, -2, -2, -2, -3])
    th_vec = np.array([np.pi/4, 0, np.pi/4, np.pi/2, 0, np.pi/4, np.pi/2, 3*np.pi/4, 0, -3*np.pi/4, -np.pi/2, -np.pi/4, 0, -np.pi/2, -np.pi/4, 0, -np.pi/4])
    lattice = np.column_stack((x_vec, y_vec, th_vec))


    L = 0.275
    v = 0.2
    u_max = np.pi/6
    m.generate_primitives(theta_init, lattice, L, v, u_max)
    m.save(file_name)

plt.figure(10, clear=True)
m.plot("b", lw = 0.5)
despine()
plt.show(block = True)
