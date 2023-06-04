# marineMechatronicsMir
code for marine mecatronics class First year Erasmus Mundus Mir, Toulon

Be carreful, all the code is in python2.
You have to change the first line of bash files in python3
and may be correct some small issues.

## Visual servoing code

The visual servoing pratical is divided into two steps :
* 2D points detection and tracking 
* visual servoing

### Install 

go to you catkin workspace :
  ```
  cd ~/catkin_ws
  cd src
  ```
 
clone the repo : 
  ```
  git clone https://github.com/cosmer-admin/autonomous_rov.git
  ```

source your catkin workspace : 
  ```
  source devel/setup.bash
  ```
### Test it on a bag

Set the env variable in you bashrc

```
 gedit ~\.bashrc
```
And edit those lines

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

#### Test the blob tracker

The blob tracker detects blob center. If you click in the image, the current blob centers are stored as desired points.
The current points are in green and the desired ones are in red.

  - The file is in script/blob_tracker_mir.py
  - The launch file is in launch/run_blob_tracker.launch


It is based on the simple blob tracker of open CV : [Find code and exaplanation here]([https://pages.github.com/](https://learnopencv.com/blob-detection-using-opencv-python-c/))
It detects and unicolor dots in an image and return their center in pixels.


The image displays 
  - the current tracked points in green 
  - the desired point in red that will be used in visual servoing

The order of the points is set at the beginning of the tracking algorithm according to the first point detected by the algorithm.

Tracked point and desired point might be ordered identically.
That is why when you reset the tracking, the desired points are also tracked. 

The tracking is monitored by clicking in the image.
  - Right click on the image to update the desired points
  - Left click on the image to initialise the tracked points and reset the desired point

![Tracking ok  : current tracked points are in green, desired points are in red](images/trackingok.png)

If there is too much offset between two successive detections, the algorithm issues a warning and the editor stops until you click on the image to reset the desired point and track.

![Tracking failed : you have to LEFT click to reinit the tracking](images/trackinko.png)


1. open a terminal and run a roscore
```
   roscore
```
2. open a new terminal and run the bag testtracking.bag with loop option so that it never stops publishing
```
  cd ~/catkin_w/src/autonomous_rov/bags
  rosbag play -l testdots.bag 
```

3. open another terminal and launch
```
  rostopic list
```
You should see the following topics

  - /br5/usb_cam/image_raw
  - /br5/usb_cam/image_raw/compressed
  - /br5/usb_cam/image_raw/compressed/parameter_descriptions


It means the robot that was used was the named br5 with a camera usb_cam and an image topic usb_cam/image_raw/compressed.
You will have to set the launch argument for the tracker to work.


4. open another terminal and run the launch file of the blob tracker
```
   roslaunch autonomous_rov run_blob_tracker.launch ns:=br5 npoints:=8 image_topic:=usb_cam/image_raw/compressed
 ````
5. look at the topic : you should see two new topics : 
  - /br5/tracked_points of type std_msgs/Float64MultiArray
  - /br5/desired_points of type std_msgs/Float64MultiArray
They are published by the node blob_tracker_mir.py. They contained the point position in the image *in pixels* : \[u1,v1,u2,v2,u3,v3,....,uN, vN\]


#### Test the orb tracker

The orb tracker detects 2D orb points and match them

  - The file is in script/orb_tracker_mir.py
  - The launch file is in launch/run_orb_tracker.launch

1. Keep the local roscore running.

2. open a new terminal and run the bag testtracking.bag with loop option so that it never stops publishing
```
  cd ~/catkin_w/src/autonomous_rov/bags
  rosbag play -l testlibrary.bag 
```

3. open another terminal and launch
```
  rostopic list
```
You should see the following topics

  - /br4/raspicam_node/image/compressed

It means the robot that was used was the named br4 with a camera raspi_cam and an image topic raspicam_node/image/compressed.
You will have to set the launch argument for the tracker to work.


4. open another terminal and run the launch file of the blob tracker. It should work the same as previously.
```
roslaunch autonomous_rov run_blob_tracker.launch ns:=br4 npoints:=8 image_topic:=raspicam_node/image/compressed
 ````

5. open another terminal and run the launch file of the orb tracker. Increase the number of tracked points.
```
roslaunch autonomous_rov run_orb_tracker.launch ns:=br4 npoints:=50 image_topic:=raspicam_node/image/compressed
 ````

6. look at the topic : you should see two new topics : 
  - /br5/tracked_points of type std_msgs/Float64MultiArray
  - /br5/desired_points of type std_msgs/Float64MultiArray
They are published by the node blob_tracker_mir.py. They contained the point position in the image *in pixels* : \[u1,v1,u2,v2,u3,v3,....,uN, vN\]


#### Test visual servoing

1. Keep roscore running

5. open another terminal and run the visual servoing launch file with number of points as an option
```
   roslaunch autonomous_rov run_visual_servoing.launch npoints:=200 ns:=br4
```
6. look at the topic : you should see two new topics : 

  - /br4/angle_degree
  - /br4/angular_velocity
  - /br4/cmd_vel
  - /br4/depth/state
  - /br4/desired_points
  - /br4/joy
  - /br4/linear_velocity
  - /br4/mavros/imu/data
  - /br4/mavros/imu/water_pressure
  - /br4/mavros/rc/override
  - /br4/raspicam_node/image/compressed
  - /br4/tracked_points
  - /br4/visual_servoing_err_norm
  - /br4/visual_servoing_error
  - /br4/visual_servoing_velocity_cam
  - /br4/visual_servoing_velocity_rob



### Real robot application

1. Set your network to 
```
IPv4 to be adress = 192.168.254.15 Netmask = 255.255.255.0 Gateway = [empty]
```
**Be carreful that your firewall is off**

2. Set the env variable in you bashrc 
```
 gedit ~\.bashrc
```
And edit those lines
```
export ROS_MASTER_URI=http://192.168.254.15:11311
export ROS_HOSTNAME=192.168.254.15
export ROS_IP=192.168.254.15
```










 
 
 

