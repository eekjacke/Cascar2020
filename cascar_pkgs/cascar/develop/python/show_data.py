#! /usr/bin/env python
"""Simple script to visualize measured cascar data"""
import sys
import matplotlib.pyplot as plt
from calib_util import BoxOff, LoadLogDataPickle
import numpy as np


def plot_data(filename):
    """Plot logged data."""
    qualisys, left, right = LoadLogDataPickle(filename, calibrate_phi=True,
                                              n_skip=0)

    print("Loaded file " + filename)
    print("  %d qualisys messages logged" % len(qualisys))
    print("  %d odometry messages from left wheel" % len(left))
    print("  %d odometry messages from right wheel" % len(right))

    if len(qualisys) > 0:
        plt.figure(10)
        plt.plot(qualisys['x'], qualisys['y'])
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Path of car')
        plt.axis('equal')
        BoxOff()

        plt.figure(11)
        plt.subplot(1, 2, 1)
        plt.plot(qualisys['t'], qualisys['x'], 'b')
        plt.plot(qualisys['t'], qualisys['y'], 'r')
        plt.xlabel('t [s]')
        plt.ylabel('[m]')
        plt.legend(['x', 'y'])
        BoxOff()

        plt.subplot(1, 2, 2)
        plt.plot(qualisys['t'], qualisys['phi']*180/np.pi)
        plt.xlabel('t [s]')
        plt.ylabel('[deg]')
        plt.title('Heading')
        BoxOff()

        plt.tight_layout()

    plt.figure(30)
    plt.subplot(2, 2, 1)
    plt.plot(right['t'], right['vel'])
    plt.xlabel('t [s]')
    plt.ylabel('velocity [%]')
    plt.title('Right wheel')
    BoxOff()

    plt.subplot(2, 2, 2)
    plt.plot(right['t'], right['steer'])
    plt.xlabel('t [s]')
    plt.ylabel('steer [%]')
    BoxOff()

    plt.subplot(2, 2, 3)
    plt.plot(right['t'], right['dt'])
    plt.xlabel('t [s]')
    plt.ylabel('dt [s]')
    BoxOff()

    plt.tight_layout()

    plt.figure(40)
    plt.subplot(2, 2, 1)
    plt.plot(left['t'], left['vel'])
    plt.xlabel('t [s]')
    plt.ylabel('velocity [%]')
    plt.title('Left wheel')
    BoxOff()

    plt.subplot(2, 2, 2)
    plt.plot(left['t'], left['steer'])
    plt.xlabel('t [s]')
    plt.ylabel('steer [%]')
    BoxOff()

    plt.subplot(2, 2, 3)
    plt.plot(left['t'], left['dt'])
    plt.xlabel('t [s]')
    plt.ylabel('dt [s]')
    BoxOff()

    plt.tight_layout()

    plt.figure(50)
    plt.plot(left['t'], left['dt'])
    plt.plot(right['t'], right['dt'])
    plt.title('Wheel tick times for left and right wheel')
    plt.xlabel('t [s]')
    plt.ylabel('dt [s]')
    plt.legend(['left', 'right'])
    BoxOff()

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Syntax: ")
        print(sys.argv[0] + " filename.pickle")
    else:
        plot_data(sys.argv[1])
