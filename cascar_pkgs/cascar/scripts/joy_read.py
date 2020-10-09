#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def cmd_callback(data):
    ax = data.axes

    print data.axes
#    lr = data.axes[0]
#    ud = data.axes[1]
#    fire = data.buttons[0]>0
#    if not fire:
#        print "ud = %.2f, lr = %.2f" % (ud, lr)
#    else:
#        print "ud = %.2f, lr = %.2f. Fire!" % (ud, lr)

def start():
    rospy.init_node('cascar_joy', anonymous=True)
    rospy.Subscriber('joy', Joy, cmd_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()

    except rospy.ROSInterruptException:
        pass
