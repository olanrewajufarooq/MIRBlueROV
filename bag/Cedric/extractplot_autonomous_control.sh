#!/bin/bash

rosbag filter p_yaw.bag p_yaw_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py p_yaw_filtered.bag 
python3 plot_autonomous_control.py "p_yaw_filtered"

rosbag filter pi_yaw.bag pi_yaw_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py pi_yaw_filtered.bag 
python3 plot_autonomous_control.py "pi_yaw_filtered"

rosbag filter pid_yaw.bag pid_yaw_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py pid_yaw_filtered.bag 
python3 plot_autonomous_control.py "pid_yaw_filtered"

rosbag filter free-path--yaw-surface.bag free-path-yaw-surface_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py free-path-yaw-surface_filtered.bag 
python3 plot_autonomous_control.py "free-path-yaw-surface_filtered"

rosbag filter free-path-yaw-surface-far.bag free-path-yaw-surface-far_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py free-path-yaw-surface-far_filtered.bag 
python3 plot_autonomous_control.py "free-path-yaw-surface-far_filtered"

rosbag filter free-path-yaw-underwater.bag free-path-yaw-underwater_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py free-path-yaw-underwater_filtered.bag 
python3 plot_autonomous_control.py "free-path-yaw-underwater_filtered" True

rosbag filter free-path-sway-surface.bag free-path-sway-surface_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py free-path-sway-surface_filtered.bag 
python3 plot_autonomous_control.py "free-path-sway-surface_filtered"

rosbag filter free-path-sway-underwater.bag free-path-sway-underwater_filtered.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state', '/br4/robot_distance']"
python3 bag2csv_python3.py free-path-sway-underwater_filtered.bag 
python3 plot_autonomous_control.py "free-path-sway-underwater_filtered" True