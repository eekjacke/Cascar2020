import numpy as np

def cost_to_go(world, x, xg):
    p_x = world.st_sp[0:2, x]
    p_g = world.st_sp[0:2, xg]
    d = np.linalg.norm(p_x-p_g)
    return d;
