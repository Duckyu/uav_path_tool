#!/usr/bin/env python
import numpy as np
import csv
import argparse

def yaw_sat(input):
    while input > np.pi:
        input -= 2*np.pi
    while input < -np.pi:
        input += 2*np.pi
    return input

def rotate(x, y, base_x, base_y, angle):
    xx = np.cos(angle) * (x - base_x) - np.sin(angle) * (y - base_y) + base_x
    yy = np.sin(angle) * (x - base_x) + np.cos(angle) * (y - base_y) + base_y
    return xx, yy

def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate infinite path')
    parser.add_argument('path_num', help='number of the path')
    parser.add_argument('--main_path', 
        type=str,
        default='/home/dklee/uav_ws/src/uav_path_tool/path/outdoor/')
    parser.add_argument('--resolution', default=200,
        help='uav speed')
    parser.add_argument('--iter', type=int, default=3,
        help='iteration path')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    path_num = args.path_num
    main_path = args.main_path
    meta_csv = open(main_path + 'meta_data.csv', 'r')
    rdr = csv.reader(meta_csv)
    uav_setup_l = []
    isInfinite = [False, False, False, False, False]
    for line in rdr:
        if line[0] == path_num:
            if line[1] == 'infinite':
                isInfinite[int(line[2])-1] = True
            uav_setup_l.append(line[1:]) # type, uav, x, y, yaw
    uav_size = len(uav_setup_l)
    if uav_size > 5:
        print("maximum 5 ERROR!!!!!")
        exit()
    print('==============')
    print('The number of uav is {}'.format(uav_size))
    print('==============')
    resolution = args.resolution
    iter = args.iter
    origin_x = []; origin_y = []; origin_yaw = []
    for i, uav in enumerate(uav_setup_l):
        origin_yaw.append(yaw_sat(float(uav[4]) * np.pi / 180.0))
        origin_x.append(float(uav[2]))
        origin_y.append(float(uav[3]))
    ################################################################
    ## you only need to edit this part variables ###################
    ################################################################
    alpha =  [ 5.0,  5.0,  5.0,  5.0,  5.0] # + : CW, - : CCW
    fix_turn = [True, True, False, False, True]
    height = [
        [3.0, 3.0],
        [3.0, 3.0],
        [3.0, 3.0],
        [3.0, 3.0],
        [3.0, 3.0]]
    ###########################################
    ### Don't edit below code #################
    ###########################################
    for l in range(uav_size):
        if not isInfinite[l]:
            print("UAV{} is not infinite path".format(l+1))
            continue
        out_csv = open(main_path + 'path{}/'.format(path_num) + 
                    'plot_uav{}.csv'.format(l+1), 'w', newline='')
        out_txt = open(main_path + 'path{}/'.format(path_num) +
                    'UAV{}.txt'.format(l+1), 'w')
        wr = csv.writer(out_csv)
        wr.writerow(['x', 'y', 'z', 'yaw'])
        out_txt.write("control\t0\n")
        out_txt.write("yaw_angle\t1\n")
        out_txt.write("cmd_type\t0\n")
        out_txt.write("pose_x	pose_y	pose_z	heading\n")
        z = height[l][0]
        Y = yaw_sat(origin_yaw[l])
        delta_z = ((height[l][1]-height[l][0])) / \
                float(resolution * iter)
        for j in range(iter):
            for i in range(resolution):
                angle = yaw_sat(2*np.pi/float(resolution)*i + np.pi/2)
                x_ = origin_x[l] + alpha[l] * np.sqrt(2) * np.cos(angle) / \
                    (np.power(np.sin(angle),2) + 1)
                y_ = origin_y[l] + alpha[l] * np.sqrt(2) * np.cos(angle) * \
                    np.sin(angle) / (np.power(np.sin(angle),2) + 1)
                x, y = rotate(x_, y_, origin_x[l], origin_y[l], origin_yaw[l])
                z += delta_z
                if not fix_turn[l]:
                    Y += 4 * np.pi / resolution
                if i==0 and j == 0:
                    out_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,0,Y))
                    wr.writerow([x, y, 0, Y])
                out_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
                wr.writerow([x, y, z, Y])
        out_csv.close()
        out_txt.close()

if __name__ == '__main__':
    main()

