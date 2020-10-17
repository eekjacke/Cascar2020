def state_transition(x, world, mp, tol = 10**-5):
    state_i = world.st_sp[:, x]
    theta_i = state_i[2]
    mprims = mp.mprims
    th = mp.th

    k = np.argwhere(np.abs((th - theta_i) % (2*np.pi)) < tol)[0][0]
    xi = []
    d = []
    u = []
    for j, mpi in enumerate(mprims[k]):
        state_next = state_i + [mpi['x'][-1], mpi['y'][-1], 0]
        state_next[2] = mpi['th'][-1]

        p = np.row_stack((mpi['x'], mpi['y'])) + state_i[0:2, None]
        if not world.in_bound(state_next) or not world.ObstacleFree(p):
            continue
        else:
            next_idx = np.argmin(np.sum((world.st_sp - state_next[:, None])**2, axis=0))
            xi.append(next_idx)
            d.append(mpi['ds'])
            u.append([k, j])
    return (xi, u, d)
