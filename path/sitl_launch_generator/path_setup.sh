#!/bin/sh
python3 make_initial_text.py >> inital_pose.xml
python3 make_sitl_gps_launch.py  >> [NAME_OF_GPS_LAUNCH].launch
python3 make_sitl_vision_launch.py >> [NAME_OF_VISION_LAUNCH].launch
