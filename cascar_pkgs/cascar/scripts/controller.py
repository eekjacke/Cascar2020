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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cascar.msg import CarCommand
import tf
from plan_path import plan_path
import time

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

def visualise_path(path_points,pub):
    path = list()
    for ii in range(len(path_points) - 1):
        loc = Pose()
        loc.position.x = path_points[ii, 0]
        loc.position.y = path_points[ii, 1]
        loc.position.z = 0
        path.append(loc)

    pose_list = list()
    path_msg = Path()

    path_msg.header.frame_id = "pather"

    for loc in path:
        pose = PoseStamped()
        pose.pose = loc
        pose_list.append(pose)
        path_msg.poses.append(pose)

    pub.publish(path_msg)

def visualise_obst(obst,pub):
    marker = Marker()
    marker.header.frame_id = "obst1"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.pose.position.x = obst[0]+obst[2]/2
    marker.pose.position.y = obst[1]+obst[3]/2
    marker.pose.position.z = 0
    marker.scale.x = obst[2]
    marker.scale.y = obst[3]
    marker.pose.orientation.w = 1
    marker.scale.z = 0.4
    marker.color.a = 1.0
    marker.color.r = 1.0    
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    pub.publish(marker)
    

def run_mpc(max_vel):

    # Init node
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('car_command', CarCommand, queue_size=1) # create object to publish commands to car
    pub_path = rospy.Publisher('pather', Path, queue_size=10) # create object to publish path to UI
    pub_car_path = rospy.Publisher('car_pather', Path, queue_size=10) # create object to publish car path to UI
    pub_obst1 = rospy.Publisher('obst1', Marker, queue_size=10) # create object to publish obst1 to UI
    pub_obst2 = rospy.Publisher('obst2', Marker, queue_size=10) # create object to publish obst2 to UI


    w = {'x': None, 'y': None, 'yaw':None, 'v':None} # Create object with car states
    obst_1 = {'x': None, 'y': None, 'yaw':None} # Create object with obst_1 states
    obst_2 = {'x': None, 'y': None, 'yaw':None} # Create object with obst_2 states
    obst_size=[0.4,0.6] # Obstacle size

    rospy.Subscriber('qualisys/obstacle_1/odom', Odometry, obst_1_callback, callback_args=obst_1)
    rospy.Subscriber('qualisys/obstacle_2/odom', Odometry, obst_2_callback, callback_args=obst_2)
    goal_obj = {'x': None, 'y': None, 'yaw':None}
    
    
    rospy.Subscriber("odom", Odometry, odom_callback, callback_args=w)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, fetch_goal, callback_args=goal_obj)

    rate = rospy.Rate(10) # desired rate in Hz
    time.sleep(1)


# Create MPC-controller
    while w['x']==None or obst_1['x']==None or obst_2['x']==None:
        print("V??ntar p?? f??rsta m??tdata")
        #print(w)
        rate.sleep()
    start = [w['x'], w['y'], w['yaw']]
    box1 = [obst_1['x'],obst_1['y'],obst_size[0],obst_size[1]]
    box2 = [obst_2['x'],obst_2['y'],obst_size[0],obst_size[1]]
    
    boxes = [box1,box2] #Obstacles

    visualise_obst(boxes[0],pub_obst1)
    visualise_obst(boxes[1],pub_obst2)
#    boxes = []
    while goal_obj['x'] == None:
        print("Set the goal point")
        rate.sleep()
    goal = [goal_obj['x'], goal_obj['y'], goal_obj['yaw']]
    
    p = plan_path(start, goal, boxes) #Path-points
    ref_path = SplinePath(p[::3]) # Create splinepath
    
    visualise_path(p,pub_path)
    

    # Controller parameters
    opts = {
        'h_p': 4,
        'gamma_d': 6,
        'gamma_theta': 1,
        'gamma_u': 0.3,
        'L': 0.275, # Wheel base
        'steer_limit': np.pi/6  # Nominally, use the same in the car
    }

    #dt = 0.1 # Sample rate
    #goal_tol = 0.1 # Goal tolerance
    mpc=ModelPredictiveControl.ModelPredictiveController(controller_params=opts, path=ref_path, goal_tol=0.006*max_vel, dt=0.1)

    traj = np.array([[w['x'], w['y']]])
    while not rospy.is_shutdown():
        if w['x']!=None:
            print(w)

            traj = np.append(traj, [[w['x'], w['y']]], axis=0)
            visualise_path(traj, pub_car_path)
            
            u=controller(w['x'],w['y'],w['yaw'],w['v'],mpc)
            # Convert angle
            u[0]=100*6/np.pi*u[0]
            print("u_steer ??r %d \n" % u[0])
            print("u_acc ??r %d \n" % u[1])

            msg = CarCommand() # create a message of the proper format

            if u[1]==0:

                msg.velocity = float(max_vel) # from -100 to 100
                msg.steer = float(u[0]) # from -100 to 100
            else:
                msg.velocity = float(-30) # from -100 to 100
                msg.steer = float(0) # from -100 to 100
                run_mpc(max_vel)
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
    max_vel=70
    try:
        run_mpc(max_vel)

    except rospy.ROSInterruptException:
        pass
