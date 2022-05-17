#!/usr/bin/env python

import rospy
import test_ros
from std_msgs.msg import Header
from termcolor import colored

def print_header_msg(h):
    print colored('Printing header message in python', 'red')
    print 'seq: ' + str(h.seq)
    print 'stamp: secs: %i nsecs: %i' % (h.stamp.secs, h.stamp.nsecs)
    print 'frame_id: ' + str(h.frame_id)

if __name__ == '__main__':
    rospy.init_node('test_header')

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # create a header message
        python_header_msg = Header()
        python_header_msg.stamp = rospy.get_rostime()
        python_header_msg.frame_id = "world"

        # print in python
        print_header_msg(python_header_msg)

        # pass to cpp
        print colored('Passing to C++', 'yellow')
        test_ros.print_header(python_header_msg)

        # get some arbitrary header from C++
        print colored('Getting from C++', 'yellow')
        cpp_header_msg = test_ros.get_arbitrary_header()

        # print the header back in python
        print_header_msg(cpp_header_msg)

        rate.sleep()
