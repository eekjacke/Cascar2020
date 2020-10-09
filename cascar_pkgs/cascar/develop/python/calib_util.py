"""Utility functions for cascar."""

import matplotlib.pyplot as plt
import pickle
import numpy as np
import pandas as pd
import scipy.io as sio


def LoadQualisysMatData(filename):
    """Load qualisys Matlab data."""
    d = loadmat(filename)
    measurementNo = [k for k in d.keys() if k.startswith('Measurement')][0]
    d = d[measurementNo]['RigidBodies']['Positions']/1000
    return d


def loadmat(filename):
    """Call low-level matlab loader and check keys."""
    data = sio.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)


def _check_keys(dict):
    """Check if entries in dictionary are mat-objects."""
    for key in dict:
        if isinstance(dict[key], sio.matlab.mio5_params.mat_struct):
            dict[key] = _todict(dict[key])
    return dict


def _todict(matobj):
    """Construct matlab objects from nested dictionaries."""
    dict = {}
    for strg in matobj._fieldnames:
        elem = matobj.__dict__[strg]
        if isinstance(elem, sio.matlab.mio5_params.mat_struct):
            dict[strg] = _todict(elem)
        else:
            dict[strg] = elem
    return dict


def BoxOff(*argin):
    """Remove ugly box around plots."""
    if len(argin) > 0:
        ax = argin[0]
    else:
        ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.xaxis.set_ticks_position('bottom')


def quaternion_to_euler_angle(w, x, y, z):
    """Convert quaternions to radians."""
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = (t2 > 1.0)*(1.0) + (t2 < 1.0)*t2
    t2 = (t2 < -1.0)*(-1.0) + (t2 > -1.0)*t2

    Y = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z


def QualisysSpeed(pos):
    """Compute velocity estimate from Qualisys data."""
    return np.append(
        1/np.diff(pos['t'])*np.sqrt(np.diff(pos['x'])**2 +
                                    np.diff(pos['y'])**2),
        0)


def OdoSpeed(odo, r=80/1000/2):
    """Compute velocity estimate from odometry data."""
    return np.pi*2*r/10/odo['dt']


def Synchronize(odo, pos, f=100, r=80/1000/2):
    """Compute synchronization time-shift for qualisys time stamps."""
    t_odo = np.arange(0, np.floor(np.max(odo['t'])*f))/f
    t_qualisys = np.arange(0, np.floor(np.max(pos['t'])*f))/f

    v_odo = np.interp(t_odo, odo['t'], OdoSpeed(odo, r))
    v_qualisys = np.interp(t_qualisys, pos['t'], QualisysSpeed(pos))

    c = np.correlate(v_odo, v_qualisys, mode='full')
    c_t = np.arange(-len(v_qualisys)+1, len(v_odo))/100
    dt = c_t[np.argmax(c)]
    return dt, c, c_t


def LoadLogDataPickle(filename, body='cascar', n_skip=10,
                      calibrate_phi=True, synchronize=True):
    """Load pickle created by cascar_logger.py."""
    with open(filename, 'rb') as h:
        data = pickle.load(h, encoding='latin1')

    if len(data['qualisys']) > 0:
        # Get qualisys data
        pos = data['qualisys'][body]['position']
        t = data['qualisys'][body]['t']
        quat = data['qualisys'][body]['orientation']

        # Find starting data for qualisys (usually second sample)
        idx = np.min(np.argwhere(t > 0))

        # Skip introducing n_skip+idx samples
        n_qualisys = np.min([len(pos), len(t), len(quat)])
        pos = pos[n_skip+idx:n_qualisys, :]
        t = t[n_skip+idx:n_qualisys]
        quat = quat[n_skip+idx:n_qualisys, :]
        _, _, phi = quaternion_to_euler_angle(quat[:, 3], quat[:, 0],
                                              quat[:, 1], quat[:, 2])
        # Collect qualisys data in data frame
        qualisys = pd.DataFrame(np.hstack((pos, phi.reshape((-1, 1)),
                                           (t-t[0]).reshape((-1, 1)))),
                                columns=['x', 'y', 'z', 'phi', 't'])

        if calibrate_phi:
            # Estimate phi-offset
            coords = qualisys[['x', 'y']].values
            phi_hat = np.zeros(coords.shape[0])
            for idx, pi in enumerate(zip(coords[0:-2], coords[1:-1])):
                phi_hat[idx] = np.arctan2(pi[1][1]-pi[0][1], pi[1][0]-pi[0][0])
            phi_hat[-1] = 0
            d_phi = np.median((phi_hat-qualisys['phi']))
            qualisys['phi'] = qualisys['phi'] + d_phi
    else:
        qualisys = []

    # Get odometry data ("wheel ticks")
    left = data['car_ticks']['left']
    right = data['car_ticks']['right']
    n_car_ticks = np.min([left.shape[0], right.shape[0]])
    left = left[n_skip:n_car_ticks]
    right = right[n_skip:n_car_ticks]

    # Save ROS original time-vector in a new column and normalize the first
#    left = np.hstack((left, left[:, 0:1]))
    left[:, 0] = left[:, 0] - left[0, 0]

#    right = np.hstack((right, right[:, 0:1]))
    right[:, 0] = right[:, 0] - right[0, 0]

    # Collect everything in data frames
    left = pd.DataFrame(left, columns=['t', 'dt', 'vel', 'steer'])
    right = pd.DataFrame(right, columns=['t', 'dt', 'vel', 'steer'])

    if len(data['qualisys']) > 0 and synchronize:
        dt, _, _ = Synchronize(left, qualisys)
        qualisys['t'] = qualisys['t'] + dt
#        qualisys['t_abs'] = qualisys['t_abs'] + dt

    return (qualisys, left, right)


def GetCalibrationSection(pos, odo, t0, t1):
    """Get section of measurement data between t0 and t1."""
    odom_calib = odo[np.logical_and(odo['t'] >= t0, odo['t'] < t1)]
    odom_calib = odom_calib.assign(
        x=np.interp(odom_calib['t'], pos['t'], pos['x']))
    odom_calib = odom_calib.assign(
        y=np.interp(odom_calib['t'], pos['t'], pos['y']))
    odom_calib = odom_calib.assign(
        phi=np.interp(odom_calib['t'], pos['t'], pos['phi']))

    # Determine initial state (x, y, phi, vx, vy, vphi)

    dp = np.diff(odom_calib[['x', 'y']][0:2], axis=0)[0]
    phi = np.arctan2(dp[1], dp[0])

    dp = np.hstack((dp, np.arctan2(dp[1], dp[0])))
    ds = np.sqrt(dp[0]**2 + dp[1]**2)
    x0 = np.hstack((odom_calib[['x', 'y']][0:1].values[0], phi, dp/ds))

    return odom_calib, x0
