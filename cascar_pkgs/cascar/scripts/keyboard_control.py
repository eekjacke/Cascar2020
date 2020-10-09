#!/usr/bin/env python3

import rospy
from rospy.rostime import get_time
from std_msgs.msg import String
from cascar.msg import CarCommand


class KeyControl():
    def __init__(self):
        rospy.init_node('key_control', anonymous=True)
        self.pub_velocity = rospy.Publisher('velocity_command', CarCommand, queue_size=10)
        self.pub_steer = rospy.Publisher('steer_command', CarCommand, queue_size=10)
        self.speed = 0.0
        self.turn = 0.0
        self.time_vel = rospy.get_time()
        self.time_steer = self.time_vel
        self.run()
        
    def engine_callback(self, data):
        msg = CarCommand()
        data = data.data
        now = rospy.get_time()
        duration = now - self.time_vel
        self.time_vel = now

        if duration > 1.0:
            time_limit = True
        else:
            time_limit = False

        if data == "forward":
            if self.speed <= 0 or time_limit:
                self.speed = 5.0
            elif self.speed > 0 and self.speed < 95:
                self.speed = self.speed + 5
            else:
                pass
        elif data == 'reverse':
            if self.speed > 0 or time_limit:
                self.speed = 0.0
            elif self.speed <= 0 and self.speed > -100:
                self.speed = self.speed - 5
            else:
                pass
        else:
            pass
        msg.velocity = self.speed
        msg.steer = 0.0
        self.pub_velocity.publish(msg)

    def steering_callback(self, data):
        msg = CarCommand()
        data = data.data
        now = rospy.get_time()
        duration = now - self.time_steer
        self.time_steer = now

        if duration > 2.0:
            time_limit = True
        else:
            time_limit = False

        if data == "right":
            if self.turn >= 0 or time_limit:
                self.turn = -20.0
            elif self.turn < 0 and self.turn > -100:
                self.turn = self.turn - 20
            else:
                pass
        elif data == 'left':
            if self.turn <= 0 or time_limit:
                self.turn = 20.0
            elif self.turn > 0 and self.turn < 100:
                self.turn = self.turn + 20
            else:
                pass
        else:
            pass
        msg.steer = self.turn
        msg.velocity = 0.0
        self.pub_steer.publish(msg)

    def run(self):
        rospy.Subscriber('velocity_key', String, self.engine_callback)
        rospy.Subscriber('steer_key', String, self.steering_callback)
        #rospy.spin()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            duration = now - self.time_vel
            #print(duration)
            if duration > 1.0:
                msg = CarCommand()
                self.pub_velocity.publish(msg)
            rate.sleep()
        

def run():
    key_cont = KeyControl()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
