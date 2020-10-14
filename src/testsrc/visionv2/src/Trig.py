#!/usr/bin/env python3

import numpy as np
from sympy import symbols, Eq, solve 

def Simple_Pinhole(P,D):
    '''
    Simple Pinhole Model to calculate physical position based on depth and pixel coordinates 
    for Zed2 left camera FullHD. Numeric.
    Assumed no rotation or translation
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    Parameters:
        P: Position (Pixel)
        D: Depth (m)
        c: Pricipal point
        f: Focal length
    '''
    f = [1058.17,1056.76]
    c = [974.63,576.225]

    xm = (P[0]-c[0])/f[0]
    ym = (P[1]-c[1])/f[1]

    x = xm*D
    y = ym*D
    return x, y

def Advanced_Pinhole(P,D):
    '''
    Advanced Pinhole Model to calculate physical position based on depth and pixel coordinates 
    for Zed2 left camera FullHD. Symbolic.
    Assumed no rotation or translation.
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        Parameters:
        P: Position (Pixel)
        D: Depth (m)
        c: Pricipal point offset
        f: Focal length
        k: Radial Distortion Coefficients
        p: Tangential Distorsion Coefficients
    '''
    f = [1058.17,1056.76]
    c = [974.63,576.225]
    k = [-0.0396842,0.00917941,-0.0047467]
    p = [0.00010877,0.000124303]

    xmm = (P[0]-c[0])/f[0]
    ymm = (P[1]-c[1])/f[1]

    xm, ym = symbols('xm ym')
    eq1 = Eq(xmm, xm*(1+k[0]*(xm^2+ym^2)+k[1]*(xm^4+ym^4)+k[2]*(xm^6+ym^6))+2*p[0]*xm*ym+p[1]*(xm^2*ym^2+2*xm^2))
    eq2 = Eq(ymm, ym*(1+k[0]*(xm^2+ym^2)+k[1]*(xm^4+ym^4)+k[2]*(xm^6+ym^6))+2*p[1]*xm*ym+p[0]*(xm^2+ym^2+2*ym^2))
    solve((eq1,eq2),(xm,ym))
    print(xm, "xm")
    print(ym, "ym")