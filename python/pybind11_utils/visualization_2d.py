#!/usr/bin/env python

def plot_grid(d, ax, colormap='jet'):
    return ax.imshow(d, cmap=colormap, origin='lower', extent=(0.0, d.shape[0], 0.0, d.shape[1]))


def plot_grid_grad(grad_d, ax):
    X = [i + 0.5 for i in range(grad_d.shape[0])]
    Y = [j + 0.5 for j in range(grad_d.shape[1])]
    U = grad_d[:, :, 0]
    V = grad_d[:, :, 1]

    return ax.quiver(X, Y, U, V, scale_units='xy')
