#!/usr/bin/env python
import numpy as np
import csv

CCW = False
center_point = [0.0, 0.0]
height = [1.0, 3.0]
radius = 1.0
start_step = 0.0
resolution = 200
iteration = 1

p = 'path/outdoor/path1/'
f = open(p + 'plot_uav1.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'z', 'yaw'])

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")
z = height[0]
delta_z = ((height[1]-height[0])) / float(resolution * iteration)
for j in range(iteration):
    for i in range(resolution):
        angle = 2*np.pi/float(resolution)*(i+start_step)
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        if CCW:
            x = center_point[0] - radius*float(np.sin(angle))
            y = center_point[1] - radius*float(np.cos(angle))
        else:
            x = center_point[0] + radius*float(np.sin(angle))
            y = center_point[1] - radius*float(np.cos(angle))
        z += delta_z
        if CCW: Y = np.pi -float(angle-np.pi/2.)
        else:   Y = -np.pi + float(angle)
        # tangent vector forward
        #   CCW: np.pi - float(angle) 
        #   CW: angle
        # tangent vector backward
        #   CCW: -angle 
        #   CW: -np.pi + float(angle)
        # normal vector inside view
        #   CCW: -float(angle-np.pi/2.) 
        #   CW: float(angle+np.pi/2.)
        # normal vector outside view
        #   CCW: np.pi -float(angle-np.pi/2.) 
        #   CW: -np.pi +float(angle+np.pi/2.)
        while Y > np.pi:
            Y -= 2*np.pi
        while Y < -np.pi:
            Y += 2*np.pi
        if i==0 and j == 0:
            print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,0,Y))
            wr.writerow([x, y, 0, Y])
        print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
        wr.writerow([x, y, z, Y])
f.close()    
