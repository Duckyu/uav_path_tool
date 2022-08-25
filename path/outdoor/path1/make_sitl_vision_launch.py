#!/usr/bin/env python3
number_of_uav = 2

print('<?xml version="1.0"?>\n<launch>\n\t<!-- MAVROS posix SITL environment launch script -->\n\t<!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->\n\t<!-- vehicle model and world -->\n\t<arg name="est" default="ekf2"/>\n\t<arg name="vehicle" default="iris"/>\n\t<arg name="model_name" default="UAV"/>\n\t<arg name="sim_model" default="gtUAV"/>\n\t<arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>\n\t<!-- gazebo configs -->\n\t<arg name="gui" default="true"/>\n\t<arg name="debug" default="false"/>\n\t<arg name="verbose" default="false"/>\n\t<arg name="paused" default="false"/>\n\t<!-- Gazebo sim -->\n\t<include file="$(find gazebo_ros)/launch/empty_world.launch">\n\t\t<arg name="gui" value="$(arg gui)"/>\n\t\t<arg name="world_name" value="$(arg world)"/>\n\t\t<arg name="debug" value="$(arg debug)"/>\n\t\t<arg name="verbose" value="$(arg verbose)"/>\n\t\t<arg name="paused" value="$(arg paused)"/>\n\t</include>')
for i in range(number_of_uav):
    f_0 = open("UAV{0}.txt".format(i+1), "r")
    d_0 = f_0.read()
    d_0 = d_0.split("\n")
    d_0 = d_0[4]
    d_0 = d_0.split("\t")
    print('\t<!-- UAV{9} -->\n\t<group ns="UAV{9}">\n\t\t<!-- MAVROS and vehicle configs -->\n\t\t<arg name="ID" value="{0}"/>\n\t\t<arg name="fcu_url" default="udp://:{1}@localhost:{2}"/>\n\t\t<!-- PX4 SITL and vehicle spawn -->\n\t\t<include file="$(find path_generator)/launch/single_gazebo_uav_spawn.launch">\n\t\t\t<arg name="x" value="{3}"/>\n\t\t\t<arg name="y" value="{4}"/>\n\t\t\t<arg name="z" value="{5}"/>\n\t\t\t<arg name="R" value="0"/>\n\t\t\t<arg name="P" value="0"/>\n\t\t\t<arg name="Y" value="{6}"/>\n\t\t\t<arg name="vehicle" value="$(arg vehicle)"/>\n\t\t\t<arg name="model_name" value="$(arg model_name)"/>\n\t\t\t<arg name="sim_model" default="$(arg sim_model)"/>\n\t\t\t<arg name="mavlink_udp_port" value="{7}"/>\n\t\t\t<arg name="mavlink_tcp_port" value="{8}"/>\n\t\t\t<arg name="ID" value="$(arg ID)"/>\n\t\t\t<arg name="gst_udp_port" value="$(eval 5600 + arg(\'ID\'))"/>\n\t\t\t<arg name="video_uri" value="$(eval 5600 + arg(\'ID\'))"/>\n\t\t\t<arg name="mavlink_cam_udp_port" value="$(eval 14530 + arg(\'ID\'))"/>\n\t\t</include>\n\t\t<!-- MAVROS -->\n\t\t<include file="$(find mavros)/launch/px4.launch">\n\t\t\t<arg name="fcu_url" value="$(arg fcu_url)"/>\n\t\t\t<arg name="gcs_url" value=""/>\n\t\t\t<arg name="tgt_system" value="$(eval 1 + arg(\'ID\'))"/>\n\t\t\t<arg name="tgt_component" value="1"/>\n\t\t</include>\n\t</group>'.format(i,14540+i,14580+i, d_0[0], d_0[1], d_0[2], d_0[3], 14560+i, 4560+i,i+1))

print("</launch>")


