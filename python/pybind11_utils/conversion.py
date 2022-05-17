#!/usr/bin/env python

import numpy as np


def grid2_point_to_numpy(p):
    return np.array([p.x, p.y])


def grid3_point_to_numpy(p):
    return np.array([p.x, p.y, p.z])


def grid3_get_center_pos(i, g):
    p = g.i2w(i)
    return [p.x + g.resolution / 2.0, p.y + g.resolution / 2.0, p.z + g.resolution / 2.0]


def grid2_get_center_pos(c, g):
    return [c[0] + g.resolution / 2.0, c[1] + g.resolution / 2.0]


def grid2_to_numpy(g):
    g_np = np.zeros((g.height, g.width))
    for i in range(g_np.shape[0]):
        for j in range(g_np.shape[1]):
            g_np[i, j] = g.get(i * g.width + j).logodds

    return g_np


def grid2_grad_to_numpy(g):
    g_np = np.zeros((g.height, g.width, 2))
    for i in range(g_np.shape[0]):
        for j in range(g_np.shape[1]):
            g_np[i, j, :] = grid2_point_to_numpy(g.get(i * g.width + j).grad)
    return g_np


def grid3_to_numpy(g):
    g_np = np.zeros((g.height, g.width, g.depth))
    for i in range(g_np.shape[0]):
        for j in range(g_np.shape[1]):
            for k in range(g_np.shape[2]):
                g_np[i, j, k] = g.get((i * g.width + j) * g.depth + k).logodds

    return g_np


def grid3_grad_to_numpy(g):
    g_np = np.zeros((g.height, g.width, g.depth, 3))
    for i in range(g_np.shape[0]):
        for j in range(g_np.shape[1]):
            for k in range(g_np.shape[2]):
                g_np[i, j, k, :] = grid3_point_to_numpy(
                    g.get((i * g.width + j) * g.depth + k).grad)
    return g_np
