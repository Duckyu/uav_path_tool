#!/usr/bin/env python
import numpy as np

round_trip = False
center_point = [0.0, 0.0]
height = [1.0, 1.0]
radius = 1.0
start_step = 0.0
resolution = 200

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")
for i in range(resolution):
    angle = 2*np.pi/float(resolution)*(i+start_step)
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    x = center_point[0] - radius*float(np.sin(angle))
    y = center_point[1] - radius*float(np.cos(angle))
    if round_trip:
        z = (height[1]-height[0])/float(resolution)*(float(resolution)-abs(float(resolution)-i*2.0))+height[0]
    else:
        z = (height[1]-height[0])/float(resolution)*i+height[0]
    Y = -float(angle-np.pi/2.)
    # tangent vector forward : angle (no)
    # tangent vector backward : -angle
    # normal vector inside view : -float(angle-np.pi/2.)
    # normal vector outside view : float(angle-np.pi/2.)
    while Y > np.pi:
        Y -= 2*np.pi
    while Y < -np.pi:
        Y += 2*np.pi
    if i==0:
        print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,0,Y))
    print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
