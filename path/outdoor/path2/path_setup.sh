#!/bin/sh
python3 make_initial_text.py >> inital_pose.xml
python3 make_sitl_gps_launch.py  >> out_path2_gps_sitl.launch
python3 make_sitl_vision_launch.py >> out_path2_vision_sitl.launch
