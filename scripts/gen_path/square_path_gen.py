#!/usr/bin/env python
from dis import dis
from tracemalloc import start
import numpy as np
import csv

CCW = True
start_point = [0.0, 0.0]
height = [3.0, 2.0]
distance = 1.0
resolution = 200
turn_ratio = 0.2
iteration = 1

p = 'path/outdoor/path1/'
f = open(p + 'plot_uav1.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'z', 'yaw'])

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")

quater_res = int(resolution/4)
delta = distance / (quater_res * (1 - turn_ratio))
delta_z = ((height[1]-height[0])) / \
            float((resolution - quater_res * (1 - turn_ratio)) * iteration)
x = start_point[0]
y = start_point[1]
z = height[0]
Y = 0 if CCW else np.pi/2
for j in range(iteration):
    for i in range(resolution):
        if i % quater_res >= quater_res * (1 - turn_ratio):
            Y += (np.pi/2) / (quater_res * turn_ratio) if CCW else -(np.pi/2) / \
                    (quater_res * turn_ratio)
        else:
            if i // quater_res == 0:
                if CCW:
                    x += delta
                else:
                    y += delta
            elif i // quater_res == 1:
                if CCW:
                    y += delta
                else:
                    x += delta
            elif i // quater_res == 2:
                if CCW:
                    x -= delta
                else:
                    y -= delta
            else:
                if CCW:
                    y -= delta
                else:
                    x -= delta
            z += delta_z

        while Y > np.pi:
            Y -= 2*np.pi
        while Y < -np.pi:
            Y += 2*np.pi
        if i==0 and j == 0:
            print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(\
                    start_point[0],start_point[1],0,Y))
            wr.writerow([x, y, 0, Y])
        print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
        wr.writerow([x, y, z, Y])
f.close()    
