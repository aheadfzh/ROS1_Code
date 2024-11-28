#!/usr/bin/env python3


import time
import subprocess

subprocess.Popen(["roslaunch","fzh_ur_pkg","load_ur10e_gazebo.launch"])
time.sleep(3)
subprocess.Popen(["roslaunch","fzh_ur_pkg","start_moveit_planning.launch"])
time.sleep(5)
subprocess.Popen(["roslaunch","fzh_ur_pkg","open_rviz.launch"])



# roslaunch ur_gazebo ur10e_bringup.launch
# roslaunch ur10e_gripper_moveit_config move_group.launch
# oslaunch ur10e_gripper_moveit_config moveit_rviz.launch  config:=true
