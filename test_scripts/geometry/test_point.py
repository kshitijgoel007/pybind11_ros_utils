#!/usr/bin/env python

import rospy
import numpy as np
from termcolor import colored
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point

import test_geometry

if __name__ == '__main__':

    # read some points from a file
    print colored('loading some points in python', 'red')
    data = np.loadtxt('./files/points.txt')
    print data

    # pass to c++ and print there
    print colored('passing to C++', 'yellow')
    point_msgs = [None] * len(data)
    for point in data:
        point_msg = Point()
        point_msg.x, point_msg.y, point_msg.z = point
        test_geometry.print_point(point_msg)
        point_msgs.append(point_msg)

    # get points from c++ and print here
    print colored('getting vector of points from C++', 'yellow')
    cpp_point_msgs = test_geometry.get_points()
    for msg in cpp_point_msgs:
        print '[%f, %f, %f]' % (msg.x, msg.y, msg.z)
