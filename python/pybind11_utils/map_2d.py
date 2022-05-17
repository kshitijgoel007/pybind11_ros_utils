#!/usr/bin/env python

import cv2
import numpy as np


def get_image(g, img_path):
    map_image = cv2.imread(img_path)
    map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
    map_image = cv2.resize(map_image, (g.width, g.height),
                           interpolation=cv2.INTER_NEAREST)
    map_image = np.flip(map_image, axis=0)

    return map_image
