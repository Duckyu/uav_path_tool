#!/usr/bin/env python
import numpy as np
import csv
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate outdoor meta data')
    parser.add_argument('--main_path', 
        type=str,
        default='/home/dklee/uav_ws/src/uav_path_tool/path/outdoor/')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    main_path = args.main_path
    meta_csv = open(main_path + 'meta_data.csv', 'w')
    wr = csv.writer(meta_csv)
    wr.writerow(['path', 'type', 'uav', 'x', 'y', 'yaw'])
    
    ## path 1
    wr.writerow([1, 'spiral', 1, 0.0, 0.0, 0.0])
    wr.writerow([1, 'spiral', 2, 0.0, 5.0, 0.0])
    wr.writerow([1, 'spiral', 3, 5.0, 0.0, 0.0])
    wr.writerow([1, 'spiral', 4, 5.0, 5.0, 0.0])
    wr.writerow([1, 'spiral', 5, 10.0, 0.0, 0.0])
    ## path 2
    wr.writerow([2, 'square', 1, 0.0, 0.0, 0.0])
    wr.writerow([2, 'square', 2, 0.0, 5.0, 0.0])
    wr.writerow([2, 'square', 3, 5.0, 0.0, 0.0])
    wr.writerow([2, 'square', 4, 5.0, 5.0, 0.0])
    wr.writerow([2, 'square', 5, 10.0, 0.0, 0.0])
    meta_csv.close()


if __name__ == '__main__':
    main()