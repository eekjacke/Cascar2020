from motion_primitives import MotionPrimitives
from define_world import BoxWorld
from lattice_planner import lattice_planner
from cost_to_go import cost_to_go
from state_transition import state_transition
import numpy as np
import sys

def plan_path(start, goal, boxes):
    m = MotionPrimitives("cascar_ws/src/opcascar2020/cascar_pkgs/cascar/scripts/mprims.pickle")
    grid_size = 0.25
    xx = np.arange(-4, 4, grid_size)
    yy = np.arange(-4, 4, grid_size)
    th = np.array([0,np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4])
    start[0] = xx[np.argmin(abs(xx-start[0]))]
    start[1] = xx[np.argmin(abs(xx-start[1]))]
    start[2] = xx[np.argmin(abs(xx-start[2]))]

    world = BoxWorld((xx, yy, th))
# a box is defined by x,y coordinates in bottom left corner and length in x,y
# direction.
    for box in boxes:
        world.add_box(box[0], box[1], box[2], box[3])

    mission = {'start': {'id': np.argmin(np.sum((world.st_sp-np.array(start)[:,None])**2,axis = 0))},
               'goal': {'id': np.argmin(np.sum((world.st_sp-np.array(goal)[:,None])**2,axis = 0))}}

    n = world.num_nodes()

    res = lattice_planner(n, mission, lambda x: state_transition(x, world, m), lambda x,xg: cost_to_go(world,x,xg))
    if np.size(res) == 0:
        sys.exit('fuck off mate, impossible task, get your boxes/end-states under control.')

    tot_path = np.array([])
    for j in range(len(res['control'])):
        node = res['plan'][j]
        controlseq = [res['control'][j]]
        path = m.control_to_path(world.st_sp[:,node],controlseq)
        if tot_path.size == 0:
            tot_path = path
        else:
            tot_path = np.concatenate((tot_path, path), axis = 0)

    return tot_path[:,0:2]
