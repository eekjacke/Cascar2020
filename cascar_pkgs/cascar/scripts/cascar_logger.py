#!/usr/bin/env python

import sys
import rospy
from cascar.msg import CarTicks
from qualisys.msg import Subject

from LogData import LogData
from datetime import datetime as dt

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = 'cascar_%s%s%s_%s_%s.pickle' % (
        dt.now().year, dt.now().month, dt.now().day, dt.now().hour, dt.now().minute)
print "Logging to file: %s" % filename

data = LogData(filename)


def logger_shutdown():
    global data

    n_ticks, n_qualisys = data.DataSize()
    print "Saving logged date into file " + filename
    print "  (%d, %d) data points (ticks) ... " % (n_ticks[0], n_ticks[0])
    print "  %s data points (qualisys) ... " % n_qualisys
    data.DumpLog()
    print "Done!"


def ticks_msg_callback(msg):
    global data
    data.AppendTicksData(msg)


def qualisys_msg_callback(msg):
    global data
    #    print "Got a qualisys message for body %s" % msg.name
    if msg.name == 'cascar' or msg.name == 'origin':
        time = rospy.Time.now().to_time()
        data.AppendQualisysData(msg, time)


def start():
    rospy.init_node('cascar_logger', anonymous=True)
    rospy.Subscriber('car_ticks', CarTicks, ticks_msg_callback)

    published_topics = rospy.get_published_topics()
    for top in published_topics:
        if top[0].startswith('/qualisys/'):
            print "Logging qualisys topic %s" % top[0]
            rospy.Subscriber(top[0], Subject, qualisys_msg_callback)

    rospy.on_shutdown(logger_shutdown)
    rospy.spin()


if __name__ == '__main__':
    try:
        start()

    except rospy.ROSInterruptException:
        pass
