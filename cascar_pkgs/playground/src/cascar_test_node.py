#!/usr/bin/env python3

import rospy
import numpy as np
from cascar.msg import CarCommand
from nav_msgs.msg import Odometry

def odom_callback(data, position):
    """Process odom data to update position."""
    position['x'] = data.pose.pose.position.x # update current position on each callback
    position['y'] = data.pose.pose.position.y


def start():
    """Start test node to show how to publish and read data."""

    rospy.init_node('test_node', anonymous=True)
    pub = rospy.Publisher('car_command', CarCommand, queue_size=1) # create object to publish commands
    position = {'x': None, 'y': None}
    rospy.Subscriber('odom', Odometry, odom_callback, callback_args=position) # subscribe to odom messages

    rate = rospy.Rate(10) # desired rate in Hz
    
    k = 0
    j = -5
    
    while not rospy.is_shutdown():
        if np.sign(k)*k == 100:
            j*=-1
        k+=j
        # publish steer command in the loop
        msg = CarCommand() # create a message of the proper format
        msg.velocity = float(50) # from -100 to 100
        msg.steer = float(k) # from -100 to 100
        pub.publish(msg) # publish the message, runs the car for about 1 second
        print(position) # optionally print position
        rate.sleep() # sleep to the next period


if __name__ == '__main__':
    try:
        start()

    except rospy.ROSInterruptException:
        pass
