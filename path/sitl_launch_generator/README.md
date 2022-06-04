# HOW TO USE
1. COPY the files in the directory which includes text path files.
	* make_initial_text.py
	* make_sitl_gps_launch.py
	* make_sitl_vision_launch.py
	* path_setup.sh
2. Change the number of the UAVs in the file.	
	* make_sitl_gps_launch.py
	* make_sitl_vision_launch.py 
		```
		number_of_uav = 5

		```
3. Change the name of launch file.	
	* path_setup.sh
		```
		#!/bin/sh
		python3 make_initial_text.py >> inital_pose.xml
		python3 make_sitl_gps_launch.py  >> [NAME_OF_GPS_LAUNCH].launch
		python3 make_sitl_vision_launch.py >> [NAME_OF_VISION_LAUNCH].launch

		```
