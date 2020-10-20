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
    
def obst_1_callback(data,obst_1):
    quat=np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    euler=tf.transformations.euler_from_quaternion(quat)
    # Update obstacle states
    obst_1['x'] = data.pose.pose.position.x
    obst_1['y'] = data.pose.pose.position.y
    obst_1['yaw'] = euler[2]
    
def obst_2_callback(data,obst_2):
    quat=np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    euler=tf.transformations.euler_from_quaternion(quat)
    # Update obstacle states
    obst_2['x'] = data.pose.pose.position.x
    obst_2['y'] = data.pose.pose.position.y
    obst_2['yaw'] = euler[2]


def run_mpc(max_vel):

    # Init node
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('car_command', CarCommand, queue_size=1) # create object to publish commands
    w = {'x': None, 'y': None, 'yaw':None, 'v':None} # Create object with car states
    obst_1 = {'x': None, 'y': None, 'yaw':None} # Create object with obst_1 states
    obst_2 = {'x': None, 'y': None, 'yaw':None} # Create object with obst_2 states
    obst_size=[0.4,0.6] # Obstacle size

    rospy.Subscriber('qualisys/obstacle_1/odom', Odometry, obst_1_callback, callback_args=obst_1)
    rospy.Subscriber('qualisys/obstacle_2/odom', Odometry, obst_2_callback, callback_args=obst_2)

    rospy.Subscriber("odom", Odometry, odom_callback, callback_args=w)

    rate = rospy.Rate(10) # desired rate in Hz
#    rate.sleep()

# Create MPC-controller
    while w['x']==None or obst_1['x']==None or obst_2['x']==None:
        print("Väntar på första mätdata")
        #print(w)
        rate.sleep()
    start = [w['x'], w['y'], w['yaw']]
    goal = [0, 0, 0]
    # Define help variables
#    a1 = obst_size[1]*np.cos(obst_1['yaw']+np.pi/2)
#    b1 = obst_size[1]*np.sin(obst_1['yaw']+np.pi/2)
#    c1 = obst_size[0]*np.cos(obst_1['yaw'])
#    d1 = obst_size[0]*np.cos(obst_1['yaw'])
#    a2 = obst_size[1]*np.cos(obst_2['yaw']+np.pi/2)
#    b2 = obst_size[1]*np.sin(obst_2['yaw']+np.pi/2)
#    c2 = obst_size[0]*np.cos(obst_2['yaw'])
#    d2 = obst_size[0]*np.cos(obst_2['yaw'])
#    if -np.pi/2<obst_1['yaw']<0:
#        box1 = [obst_1['x'],obst_1['y']-d1,a1+c1,b1+d1]
#    elif 0<=obst_1['yaw']<np.pi/2:
#        box1 = [obst_1['x']-a1,obst_1['y'],a1+c1,b1+d1]
#    else :
#        print("Bad angle on obstacle 1 you moron")
#    
#    if -np.pi/2<obst_2['yaw']<0:
#        box2 = [obst_2['x'],obst_2['y']-d2,a2+c2,b2+d2]
#    elif 0<=obst_2['yaw']<np.pi/2:
#        box2 = [obst_2['x']-a2,obst_2['y'],a2+c2,b2+d2]
#    else :
#        print("Bad angle on obstacle 2 you moron")

    box1 = [obst_1['x'],obst_1['y'],obst_size[0],obst_size[1]]
    box2 = [obst_2['x'],obst_2['y'],obst_size[0],obst_size[1]]
    boxes = [box1,box2] #Obstacles
    p = plan_path(start, goal, boxes) #Path-points
    p=p[::5]
    print(p)
    ref_path = SplinePath(p) # Create splinepath
    # print(ref_path)

    # Controller parameters
    opts = {
        'h_p': 10,
        'gamma_d': 10,
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
    max_vel=50
    try:
        run_mpc(max_vel)

    except rospy.ROSInterruptException:
        pass
