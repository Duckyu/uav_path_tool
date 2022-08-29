# Simple UAV Path Tool
## Contents
* Most of the function generated with python
* Pure pursuit & LoS path follower implemented
* Multi uav SITL launch file auto-generation shell script based on the path text file (look at this [file](path/sitl_launch_generator/README.md))
  * GPS based
  * External pose estimation based
* SITL launch file can generate two type of airframe
  * Oridunary iris
  * External pose estimation source based iris
* [Convertion code](scripts/gt_parse_vision.py) for groundtruth from GAZEBO simulator and optitrack motion tracker (not tested yet) to mavros vision topic (mavros/vision_pose/pose)
* [Spiral path generator](scripts/spiral_path_gen.py) included
* Finite State Machine based controller adopted.

## Prerquisition
* Python3
* ROS
* GAZEBO
* MAVROS Package
* PX4 firmware setup with ROS package registration
* [Custom airframe](PX4_custom_airframe/10032_gtUAV) has to be located in the proper directory($HOME/.ros/etc/init.d-posix/airframes)
  * After the file located in the proper directory, you need to make PX4 firmware again. Please refer to this [web](https://docs.px4.io/master/en/dev_airframes/adding_a_new_frame.html).

## Path text file rule([example file](path/indoor/path1/UAV.txt))
* First Line: write down type of control (0: pure pursuit, 1: LoS, 2: translation position)
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
* Simple example
  ```bash
  roslaunch path_generator in_path1_gps_sitl.launch
  ```
  ```bash
  roslaunch path_generator test_gps_sitl.launch file_dir:="$(rospack find path_generator)/path/indoor/path1"
  ```

* External pose estimation source usage(in this example, GT pose data from gazebo has been used for the external pose estimation)
  ```bash
  roslauch path_generator in_path1_vision_sitl.launch
  ```
  ```bash
  roslaunch path_generator test_vision_sitl.launch file_dir:="$(rospack find path_generator)/path/indoor/path1"
  ```
  Due to the difference between GPS based flight and external pose estimation source based flight, I've made two types of simulation.
  
## Caution
  For vision based multi sitl, it won't work due to large amount of computation on simulator.
 
  Therefore, you need to use only one vision based controller per simulation task.

## Trouble shooting
* Import error while using the command ```rosrun``` or ```roslaunch```
  * Build this package with ```catkin build``` command
  * USE ```chmod +x import_error_solver.sh && ./import_error_solver.sh```)

* If you are not available to make drone fly, please check the PX4 firmware. Stable version of the firmware can be installed by the below command.(Stable version has been tested by [EungChang-Mason-Lee](https://github.com/engcang/mavros-gazebo-application#installation))
  ```console
  $ cd PX4-Autopilot
  $ git reset --hard 96c7fe4978bab2af970a097f4898e024c2d33440
  $ git submodule update --init --recursive
  ```
  
* If error "/usr/bin/env: ‘python’: No such file or directory" occurs, youhave to link the command python.
  * If you are using UBUNTU 18.04,
  ```bash
  ln -s /usr/bin/python2 /usr/bin/python
  ```
  * If you are using UBUNTU 20.04,
  ```bash
  ln -s /usr/bin/python3 /usr/bin/python
  ```
