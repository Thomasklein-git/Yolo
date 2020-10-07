#!/usr/bin/env python3

from scipy.spatial import distance as dist
import numpy as np
from Object_handler import Object_handler
#import Utils


# 2*arctan(pixelNumber/(2*focalLength)) * (180/pi)
# https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
FOV_V = 71.5
FOV_H = 104
OH = Object_handler(20)
P = OH.D2P(400,400,10)

