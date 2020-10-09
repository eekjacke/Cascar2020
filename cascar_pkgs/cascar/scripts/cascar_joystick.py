#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from cascar.msg import CarCommand
from std_srvs.srv import Empty

def cmd_callback(data, cmd):
    """Process joystick data."""

    lr = data.axes[0]
    ud = data.axes[1]
    fire = data.buttons[0] > 0
    cmd['velocity'] = ud*100
    cmd['steer'] = lr*100
    cmd['new'] = True

    lidarMotorOn = data.axes[5]>0.5
    lidarMotorOff = data.axes[5]<-0.5

    if fire:
        srv_zero = rospy.ServiceProxy('set_origin', Empty)
        srv_zero()
    if lidarMotorOn:
        srv_lidarOn = rospy.ServiceProxy('start_motor', Empty)
        srv_lidarOn()
    if lidarMotorOff:
        srv_lidarOff = rospy.ServiceProxy('stop_motor', Empty)
        srv_lidarOff()


def start():
    """Start the joystick controller node."""
    cmd = {'velocity': None, 'steer': None, 'new': False}
    rospy.init_node('cascar_joy', anonymous=True)
    rospy.Subscriber('joy', Joy, cmd_callback, callback_args=cmd)
    pub = rospy.Publisher('car_command', CarCommand, queue_size=10)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if cmd['new']:
            msg = CarCommand()
            msg.velocity = cmd['velocity']
            msg.steer = cmd['steer']
            pub.publish(msg)
            # cmd['new'] = False
        else:
            pass  # print "No command"

        rate.sleep()


if __name__ == '__main__':
    try:
        start()

    except rospy.ROSInterruptException:
        pass
