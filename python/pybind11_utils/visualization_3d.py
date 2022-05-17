#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import pybind11_utils.conversion as conv

import numpy as np

def visualize_gradient(grad_grid, ns='grad_grid', frame_id='world'):
    ret = MarkerArray()
    t = rospy.Time.now()
    for i in range(grad_grid.height):
        for j in range(grad_grid.width):
            for k in range(grad_grid.depth):
                idx = (i * grad_grid.width + j) * grad_grid.depth + k
                v = grad_grid.get(idx)

                m = Marker()
                m.header = Header()
                m.header.stamp = t
                m.header.frame_id = frame_id
                m.ns = ns
                m.id = idx
                m.type = m.ARROW
                m.action = m.ADD

                m.scale.x = 0.01
                m.scale.y = 0.015
                m.scale.z = 0.0
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 1.0

                if v.logodds > 0.5 * grad_grid.resolution:
                    vec = conv.grid3_point_to_numpy(v.grad) * 0.5 * grad_grid.resolution / v.logodds
                else:
                    vec = conv.grid3_point_to_numpy(v.grad)

                cell_center = np.array(conv.grid3_get_center_pos(idx, grad_grid))
                m.points.append(Point(cell_center[0], cell_center[1], cell_center[2]))
                m.points.append(Point(cell_center[0] + vec[0], cell_center[1] + vec[1], cell_center[2] + vec[2]))
                
                ret.markers.append(m)

    return ret
