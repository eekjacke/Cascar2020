#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 09:41:59 2020

@author: gustav
"""
import rospy
import ModelPredictiveControl
from splinepath import SplinePath
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cascar.msg import CarCommand
import tf
from plan_path import plan_path

def odom_callback(data,w):
    quat=np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    euler=tf.transformations.euler_from_quaternion(quat)
    v=np.sqrt(data.twist.twist.linear.x**2+data.twist.twist.linear.y**2)
    # Update car states
    w['x'] = data.pose.pose.position.x
    w['y'] = data.pose.pose.position.y
    w['yaw'] = euler[2]
    w['v'] = v

def fetch_goal(data, goal_obj):
    quat = np.array([
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w
    ])
    euler = tf.transformations.euler_from_quaternion(quat)
    goal_obj['x'] = data.pose.position.x
    goal_obj['y'] = data.pose.position.y
    goal_obj['yaw'] = euler[2]
    print(goal_obj)

def run_mpc(max_vel):

    # Init node
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('car_command', CarCommand, queue_size=1) # create object to publish commands
    w = {'x': None, 'y': None, 'yaw':None, 'v':None} # Create object with car states
    goal_obj = {'x': None, 'y': None, 'yaw':None}

    rospy.Subscriber("odom", Odometry, odom_callback, callback_args=w)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, fetch_goal, callback_args=goal_obj)

    rate = rospy.Rate(10) # desired rate in Hz
#    rate.sleep()

    while goal_obj['x'] == None:
        print("Set the goal point")
        rate.sleep()

# Create MPC-controller
    start = [w['x'], w['y'], w['yaw']]
    goal = [goal_obj['x'], goal_obj['y'], goal_obj['yaw']]
    boxes = [[-2,-2,6,1],[-2,-2,1,4],[-2,1,4,1]]
    p = plan_path(start, goal, boxes) #Path-points
    ref_path = SplinePath(p) # Create splinepath

    rospy.Publisher('/path', p, queue_size = 10)

    # Controller parameters
    opts = {
        'h_p': 10,
        'gamma_d': 1,
        'gamma_theta': 1,
        'gamma_u': 1,
        'L': 0.275, # Wheel base
        'steer_limit': np.pi/6  # Nominally, use the same in the car
    }

    #dt = 0.1 # Sample rate
    #goal_tol = 0.1 # Goal tolerance
    mpc=ModelPredictiveControl.ModelPredictiveController(controller_params=opts, path=ref_path, goal_tol=0.006*max_vel, dt=0.1)

    while not rospy.is_shutdown():
        if w['x']!=None:
            print(w)
            u=controller(w['x'],w['y'],w['yaw'],w['v'],mpc)
            # Convert angle
            u[0]=100*6/np.pi*u[0]
            print("u_steer är %d \n" % u[0])
            print("u_acc är %d \n" % u[1])

            msg = CarCommand() # create a message of the proper format

            if u[1]==0:

                msg.velocity = float(max_vel) # from -100 to 100
                msg.steer = float(u[0]) # from -100 to 100
            else:
                print("Är framme")
                msg.velocity = float(0) # from -100 to 100
                msg.steer = float(0) # from -100 to 100
            print("Skickar meddelande")
            pub.publish(msg) # publish the message, runs the car for about 1 second
            rate.sleep()


def controller(x,y,yaw,v,mpc):
    # Import data from visionen
    w=np.array([x, y, yaw, v])
#    w=np.array([-1,0,0,2])

    if mpc.run(w):
        u=mpc.u(w)
    else:
        u=np.array([0,1])
    return u

if __name__ == '__main__':
    max_vel=100
    try:
        run_mpc(max_vel)

    except rospy.ROSInterruptException:
        pass
