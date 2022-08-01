#!/bin/sh
python3 make_initial_text.py >> inital_pose.xml
python3 make_sitl_gps_launch.py  >> in_path6_gps_sitl.launch
python3 make_sitl_vision_launch.py >> in_path6_vision_sitl.launch
