#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from rospy.rostime import get_time
from std_srvs.srv import Empty, EmptyResponse

from cascar_main import CasCar
from cascar.msg import CarCommand

def set_origin_callback(request, car):
    print("Reset origin of car")
    car.zero_position()
    return EmptyResponse()

def cmd_callback(data, car):
    """Callback function for handling CarCommand messages"""
    vel = int(data.velocity)
    steer = int(data.steer)
    vel = np.min([np.max([vel, -100]), 100])
    steer = np.min([np.max([steer, -100]), 100])

    if car.is_connected():
        # rospy.loginfo('CarAct: [' + rospy.get_caller_id() +
        # ']: v=%f, s=%f', vel, steer)
        car.command_car(vel, steer)
    else:
        print("Something is wrong with serial connection")  
        
def velocity_callback(data, car):
    vel = int(data.velocity)
    vel = np.min([np.max([vel, -100]), 100])
    if car.is_connected():
        car.command_velocity(vel)
    else:
        print("Something is wrong with serial connection")

def steer_callback(data, car):
    steer = int(data.steer)
    steer = np.min([np.max([steer, -100]), 100])
    if car.is_connected():
        car.command_steer(steer)
    else:
        print("Something is wrong with serial connection")

def start_car():
    rospy.init_node('Cascar', anonymous=True)
    car = CasCar()
    if not car.is_connected():
        print("Connection problems with car, exiting ...")
        exit()
    
    rospy.Subscriber('car_command', CarCommand, cmd_callback, callback_args=car)
    rospy.Subscriber('velocity_command', CarCommand, velocity_callback, callback_args=car)
    rospy.Subscriber('steer_command', CarCommand, steer_callback, callback_args=car)
    rospy.Service('set_origin', Empty, lambda x: set_origin_callback(x, car))

    # Wave with wheels to indicate that car is alive
    print("Waiting for communications to settle (3 sec) ... ")
    rospy.sleep(3)
    print("Waving with wheels ... ")
    car.command_steer(-50)
    rospy.sleep(0.4)
    car.command_steer(50)
    rospy.sleep(0.4)
    car.command_steer(0)

    # Start the car
    print("Car is ready to operate ... ")
    car.start()


if __name__ == '__main__':
    try:
        start_car()

    except rospy.ROSInterruptException:
        pass
