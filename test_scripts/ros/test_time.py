#!/usr/bin/env python

import rospy
import test_ros
from termcolor import colored

if __name__ == '__main__':
    rospy.init_node('test_time')

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # get the current time and print it inside python
        current_time = rospy.get_rostime()
        rospy.loginfo("Getting time from python %i %i",
                      current_time.secs, current_time.nsecs)

        # pass this time into the cpp wrapper for ros::Time and print it there
        print colored('Passing to C++', 'yellow')
        test_ros.print_time(current_time)

        # get the time from C++
        print colored('Getting from C++', 'yellow')
        time_from_cpp = test_ros.get_arbitrary_time()

        # print the time from C++ here
        rospy.loginfo("Printing time in python %i %i",
                      time_from_cpp.secs, time_from_cpp.nsecs)

        rate.sleep()
