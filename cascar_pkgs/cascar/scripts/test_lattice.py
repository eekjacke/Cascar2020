import lattice_planner.handle_mp.motion_primitives as mp
import numpy as np

m = mp.MotionPrimitives("lattice_planner/mprims.pickle")

xx = np.arange(-4, 4)
yy = np.arange(-4, 4)
th = np.array([0,np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4])

world = BoxWorld((xx, yy, th))

world.add_box(0, 0, 2, 2)

start = [-3, -3, 0]
goal = [3, 3, np.pi/2]

mission = {'start': {'id': np.argmin(np.sum((world.st_sp-np.array(start)[:,None])**2,axis = 0))},
           'goal': {'id': np.argmin(np.sum((world.st_sp-np.array(goal)[:,None])**2,axis = 0))}}

n = world.num_nodes()

res = lattice_planner(n, mission, lambda x: state_transition(x, world, m))
