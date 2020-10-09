#!/usr/bin/env python3

import serial
import numpy as np
import os
import rospy
from time import sleep
from cascar.msg import CarTicks
from cascar.msg import CarSensor

wheel_radius = 78.0/2/1000  # [m]
n_magnets = 10

steer_par = [0.02840205, 0.00335286]

def steer_to_rad(steer):
    return steer_par[0] + steer*steer_par[1]

class CasCar:
    # Max speed = 1 m/s => (approx) 40 ticks/sec per wheel => fs=100 hz (margin)
    fs = 100  # Main frequency

    SER_PORTS = ['/dev/arduino']
    SER_RATE = 115200
    ser = None
    curr_steer = 0.0
    curr_vel = 0.0

    def __init__(self):
        self.open_serial_connection()
        self.car_pub = rospy.Publisher('sensor/cascar', CarSensor, queue_size=10)
        self.log = rospy.get_param('~log', False)
        
        if self.log:
            print("Broadcast of logging messages activated")
            self.log_pub = rospy.Publisher('car_ticks', CarTicks, queue_size=100)
        else:
            print("Broadcast of logging messages not activated")

    def start(self):
        rate = rospy.Rate(self.fs)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            while self.data_waiting():
                wheel, deltaT = self.get_odom_msg()
                if deltaT and wheel == 'R':
                    # Odometry message received, only consider right wheel
                    sgn = np.sign(self.curr_vel) if self.curr_vel != 0 else 1
                    ds = np.pi*2*wheel_radius/n_magnets * sgn
                    msg = CarSensor()
                    msg.header.stamp = rospy.Time.now()
                    msg.v = ds/deltaT
                    msg.df = steer_to_rad(self.curr_steer)
                    
                    self.car_pub.publish(msg)

                if self.log:
                    self.publish_car_ticks(wheel, deltaT, current_time)
            rate.sleep()

    def publish_car_ticks(self, w, deltaT, time):
        msg = CarTicks(wheel=w, dt=deltaT,
                       steer=self.curr_steer,
                       velocity=self.curr_vel,
                       t=time.to_time())
        self.log_pub.publish(msg)

    def is_connected(self):
        return self.ser

    def command_velocity(self, vel):
        if self.ser and self.ser.writable():
            v = np.min([np.max([vel, -100]), 100])
            command = 'T;{0}\r'.format(int(v))
            self.ser.write(command.encode())
            self.curr_vel = v

    def command_steer(self, steer):
        if self.ser and self.ser.writable():
            s = np.min([np.max([steer, -100]), 100])
            command = 'S;{0}\r'.format(int(s))
            self.ser.write(command.encode())
            self.curr_steer = s

    def command_car(self, vel, steer):
        if self.ser and self.ser.writable():
            v = np.min([np.max([vel, -100]), 100])
            s = np.min([np.max([steer, -100]), 100])
            command = 'T;{0}\rS;{1}\r'.format(int(v), int(s))
            self.ser.write(command.encode())
            self.curr_steer = s
            self.curr_vel = v

    def open_serial_connection(self):
        for p in self.SER_PORTS:
            if os.path.exists(p):
                ser = serial.Serial(p, self.SER_RATE)
                ser.flush()
                print("Waiting for serial port " + p + " to wake up ... ")
                sleep(2)
                ser.write('PING;\r'.encode())
                sleep(0.05)
                foundSer = False
                msg = ''
                while ser.inWaiting() > 0:
                    print(msg)
                    msg = ser.readline().decode()
                    msg = msg.split(';')[0]
                    if msg == 'PONG':
                        foundSer = True

                if not foundSer:
                    print('Car NOT connected to ' + ser.name)
                    ser.close()
                else:
                    print('Car connected to ' + ser.name)
                    self.ser = ser
                    return

    def data_waiting(self):
        return self.ser.in_waiting > 0

    def get_odom_msg(self):
        if self.ser and self.ser.readable():
            msg = self.ser.readline().decode()
            try:
                wheel, deltaT = msg.split(';')
                if wheel in ['R', 'L']:
                    deltaT = float(deltaT)/1e6
                    if deltaT > 0:
                        return (wheel, deltaT)
                    else:
                        return (None, None)
            except Exception as err:
                print("Strange message: " + msg)
                return (None, None)
