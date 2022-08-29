#!/usr/bin/env python
import numpy as np
import csv

center_point = [0.0, 0.0]
height = [3.0, 3.0]
alpha = 2.0
resolution = 200
iteration = 3

p = 'path/outdoor/path15/'
f = open(p + 'plot_uav1.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'z', 'yaw'])

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")
###########Lemniscate of Bernoulli##########
z = height[0]
delta_z = ((height[1]-height[0])) / float(resolution * iteration)
for j in range(iteration):
    for i in range(resolution):
        angle = 2*np.pi/float(resolution)*i + np.pi/2
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        x = center_point[0] + alpha * np.sqrt(2) * np.cos(angle) / (np.power(np.sin(angle),2) + 1)
        y = center_point[1] + alpha * np.sqrt(2) * np.cos(angle) * np.sin(angle) / (np.power(np.sin(angle),2) + 1)
        z += delta_z
        Y = 0
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