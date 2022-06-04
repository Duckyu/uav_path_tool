# Simple UAV Path Tool
## Contents
* Most of the function generated with python
* Pure pursuit & LoS path follower implemented
* Multi uav SITL launch file auto-generation shell script based on the path text file (look at this [file](path/sitl_launch_generator/README.md))
  * GPS based
  * Vision based
* SITL launch file can generate two type of airframe
  * Oridunary iris
  * external odometry source based iris
* Convertion code for groundtruth from GAZEBO simulator and optitrack motion tracker(not tested yet) to mavros vision topic
* [Spiral path generator](scripts/spiral_path_gen.py) included

## Prerquisition
* ROS
* GAZEBO
* MAVROS Package
* PX4 firmware setup with ROS package registration
* custom airframe has to be located in the proper directory($HOME/.ros/etc/init.d-posix/airframes)
  * After the file located in the proper directory, you need to make PX4 firmware again. Please refer to this [web](https://docs.px4.io/master/en/dev_airframes/adding_a_new_frame.html).

## Path text file rule([example file](path/indoor/path1/uav0.txt))
* First Line: write down type of control (0: pure pursuit, 1: LoS)
  ```
  control	1
  ```
* Second Line: write down type of yaw rotation (0: angle position control, 1: angle trapezoidal rate control)
  ```
  yaw_angle	0
  ```
* Third Line: notation for data
  ```
  pose_x	pose_y	pose_z	heading
  ```
* Rest of the Lines: 4 DoF coordinate (x, y, z, yaw) with "\t" spliter
  ```
  0.300000	0.300000	0.000000	0.000000
  ```
## Example
```
roslauch path_generator path1_test_sitl_gps.launch
```
```
roslauch path_generator test_gps_sitl.launch file_dir:="$(rospack find path_generator)/path/indoor/path1"
```
## Trouble shooting
* Import error while using the command ```rosrun``` or ```roslaunch```
	Build this package with ```catkin build``` command
	USE ```chmod +x import_error_solver.sh && ./import_error_solver.sh```)
	
  
