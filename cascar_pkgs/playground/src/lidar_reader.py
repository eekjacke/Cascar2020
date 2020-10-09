#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarReader:
    """ Toy example class of how to read and extract laser scan data."""
    def __init__(self):
        rospy.init_node("lidar_reader", anonymous=True)
        self.pub = rospy.Publisher("/cartesian_scan", Float32MultiArray, queue_size=10)
        self.angles = np.linspace(-np.pi, np.pi, num=360)
        self.ranges = []
        self.run()

    def polar_to_cartesian(self, ranges):
        """ With lidar range data as input, covert to cartesian points. """
        Y = ranges*np.cos(self.angles)
        X = ranges*np.sin(self.angles)
        return (X,Y)
    
    def callback(self, scan):
        """ Once lidar scans are available. Convert and publish as a float array. """
        ranges = scan.ranges
        X,Y = self.polar_to_cartesian(ranges)
        array = np.array([X,Y]).reshape(-1) # This will be a 720 long array: 360 X, 360 Y...
        msg = Float32MultiArray(data=array)
        self.pub.publish(msg)    
    
    def run(self):
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.spin()
        
if __name__ == '__main__':
    try:
        reader_instance = LidarReader()
    except rospy.ROSInterruptException:
        pass        