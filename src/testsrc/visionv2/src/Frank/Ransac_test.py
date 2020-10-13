#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from skimage.measure import ransac, LineModelND
import math
import cv2

def D2P(center_x,center_y,Depth):
    # Converts the distance and pixel values of a single point to a
    # physical coordinate
    # Camera constants for ZED2
    fx=1058.17
    fy=1056.76
    cx=108
    cy=78
        
    # Boundary box center and Depth 
    x = center_x
    y = center_y
    D = Depth

    Z = (fx*fy*D)/math.sqrt(cx**2*fy**2 - 2*cx*fy**2*x + cy**2*fx**2 - 2*cy*fx**2*y + fx**2*fy**2 + fx**2*y**2 + fy**2*x**2)
    X = (x-cx)*Z/fx
    Y = (y-cy)*Z/fy
    P = X, Y, Z
    return X, Y, Z

img = cv2.imread("depth20.png",-1)/(2**16)
#pointCloud
#fig = plt.figure()
#ax
X = []
Y = []
Z = []

for i in range(0,len(img)):
    for j in range(0,len(img[i])):
        Xc, Yc, Zc = D2P(j,i,img[i][j])
        X.append(Xc)
        Y.append(Yc)
        Z.append(Zc)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X,Y,Z,marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()

#cv2.imshow("img", img)
#cv2.waitKey(50)
#model = LineModelND()
#model.estimate(img)

#model_robust, inliers = ransac(img, LineModelND, min_samples=2,
#                               residual_threshold=10, max_trials=1000)
#outliers = inliers == False

#cv2.destroyAllWindows

