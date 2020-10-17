from motion_primitives import MotionPrimitives
from define_world import BoxWorld
from lattice_planner import lattice_planner
from cost_to_go import cost_to_go
from state_transition import state_transition
import matplotlib.pyplot as plt
from seaborn import despine


import numpy as np

m = MotionPrimitives("mprims.pickle")

xx = np.arange(-4, 4, 0.4)
yy = np.arange(-4, 4, 0.4)
th = np.array([0,np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4])

world = BoxWorld((xx, yy, th))

world.add_box(0, 0, 2, 2)
world.add_box(-2, -2, 5, 4)

start = [-3, -3, 0]
goal = [3, 3, np.pi/2]

mission = {'start': {'id': np.argmin(np.sum((world.st_sp-np.array(start)[:,None])**2,axis = 0))},
           'goal': {'id': np.argmin(np.sum((world.st_sp-np.array(goal)[:,None])**2,axis = 0))}}

n = world.num_nodes()

res = lattice_planner(n, mission, lambda x: state_transition(x, world, m), lambda x,xg: cost_to_go(world,x,xg))

print(res)

plt.figure(10, clear=True)
world.draw()
for j in range(len(res['control'])):
    node = res['plan'][j]
    controlseq = [res['control'][j]]
    path = m.control_to_path(world.st_sp[:,node],controlseq)
    xaxis = path[:,0]
    yaxis = path[:,1]
    plt.plot(xaxis, yaxis, color = 'lightblue')
despine()
plt.show(block = True)
