#!/bin/sh
python3 make_initial_text.py >> inital_pose.xml
python3 make_sitl_gps_launch.py  >> path3_test_sitl_gps.launch
python3 make_sitl_vision_launch.py >> path3_test_sitl_vision.launch
