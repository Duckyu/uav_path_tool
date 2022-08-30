#!/usr/bin/env python
import numpy as np
import csv

CW = True
center_point = [0.0, 0.0]
height = [3.0, 3.0]
radius = 2.0
start_step = 0.0
resolution = 200
iteration = 3

# 엑셀 파일을 csv로 열어서 UAV 개수와 UAV 시작 위치 및 시작 yaw를 받아옴.


path_path = 'path/outdoor/path1/'
file_csv = open(path_path + 'plot_uav1.csv', 'w', newline='')
file_txt = open(path_path + 'UAV1.txt', 'w')
wr = csv.writer(file_csv)
wr.writerow(['x', 'y', 'z', 'yaw'])

file_txt.write("control\t0")
file_txt.write("yaw_angle\t1")
file_txt.write("cmd_type\t0")
file_txt.write("pose_x	pose_y	pose_z	heading")
z = height[0]
delta_z = ((height[1]-height[0])) / float(resolution * iteration)
for j in range(iteration):
    for i in range(resolution):
        angle = 2*np.pi/float(resolution)*(i+start_step)
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        if CW:
            x = center_point[0] - radius*float(np.sin(angle))
            y = center_point[1] - radius*float(np.cos(angle))
        else:
            x = center_point[0] + radius*float(np.sin(angle))
            y = center_point[1] - radius*float(np.cos(angle))
        z += delta_z
        if CW: Y = np.pi - float(angle) 
        else:   Y = angle
        # tangent vector forward
        #   CW: np.pi - float(angle) 
        #  CCW: angle
        # tangent vector backward
        #   CW: -angle 
        #  CCW: -np.pi + float(angle)
        # normal vector inside view
        #   CW: -float(angle-np.pi/2.) 
        #  CCW: float(angle+np.pi/2.)
        # normal vector outside view
        #   CW: np.pi -float(angle-np.pi/2.) 
        #  CCW: -np.pi +float(angle+np.pi/2.)
        while Y > np.pi:
            Y -= 2*np.pi
        while Y < -np.pi:
            Y += 2*np.pi
        if i==0 and j == 0:
            file_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,0,Y))
            wr.writerow([x, y, 0, Y])
        file_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
        wr.writerow([x, y, z, Y])
file_csv.close() 
file_txt.close()    
