#!/bin/bash

rosbag filter 1_3.bag 1_3_solution.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state']"
python3 bag2csv_python3.py 1_3_solution.bag 
python3 extract_and_plot_all.py "1_3_solution"

rosbag filter 1_4.bag 1_4_solution.bag "topic in ['/br4/angle_degree', '/br4/mavros/rc/override', '/br4/angular_velocity', '/br4/mavros/state', '/br4/depth/state']"
python3 bag2csv_python3.py 1_4_solution.bag
python3 extract_and_plot_all.py "1_4_solution"