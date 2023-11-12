#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import (Int8, Int16, Float64, Empty, Int16MultiArray, Float64MultiArray, String)
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import (Joy, Imu, FluidPressure, LaserScan)
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


###---- Visual Tracking and Servoing----
from sensor_msgs.msg import CompressedImage
import cv2
import copy
import time
import sys
import argparse

# ---------- Global Variables ---------------
global nb_points_vs
global reset_desired_points
global desired_points
global desired_keypoints

nb_points_vs = 5
reset_desired_points = True
desired_points = []
flag_alert = False
desired_keypoints = []

# Create SimpleBlobDetector
detector = cv2.SimpleBlobDetector_create()

def cameracallback(image_data):
    global nb_points_vs
    global reset_desired_points
    global desired_points
    global desired_keypoints

    # get image data
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    image_np_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
   
    position = (10,30)
    text = "Click on the image : reset desired point."
    cv2.putText(image_np,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1)

    # Detect blobs in the image
    keypoints = detector.detect(image_np_gray)

    # img_with_keypoints = cv2.drawKeypoints(image_np, keypoints, np.array([]), (0, 0, 255),
                                        #    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Draw circles and dots around detected blobs
    for i, kp in enumerate(keypoints):
        x = int(kp.pt[0])
        y = int(kp.pt[1])
        radius = int(kp.size / 2)
        cv2.circle(image_np, (x, y), radius, (0, 255, 0), 2)
        cv2.circle(image_np, (x, y), 2, (0, 0, 255), -1)
        cv2.putText(image_np, str(i+1), (x+5, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.namedWindow("image")
    
    # Draw circles for desired points
    for i, dp in enumerate(desired_keypoints):
        x = int(dp.pt[0])
        y = int(dp.pt[1])
        radius = int(dp.size / 2)
        cv2.circle(image_np, (x, y), radius, (255, 255, 0), 2)
        cv2.circle(image_np, (x, y), 2, (255, 0, 255), -1)
        cv2.putText(image_np, str(i+1), (x+5, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # cv2 mouse 
    cv2.setMouseCallback("image", click_detect, keypoints)   
    cv2.imshow("image", image_np)
    cv2.waitKey(2)



def click_detect(event, x, y, flags, param):
    global reset_desired_points, desired_points, pub_desired_point, desired_keypoints

    keypoints = param
    if event == cv2.EVENT_LBUTTONDOWN:
        reset_desired_points = True
        print("Desired points will be updated.")

    # Store and publish current keypoints as desired points
    if reset_desired_points:
        reset_desired_points = False
        desired_keypoints = keypoints
        desired_points = []
        for kp in keypoints:
            desired_points.append(kp.pt)
        # print("Desired points updated:", desired_points)


def subscriber():
    #camera
    rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, cameracallback,  queue_size = 1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':
    
    rospy.init_node('blob_tracker_mir', anonymous=False)  
    
    print ('tracker launched')
    
    if rospy.has_param('~points'):
        nb_points_vs = rospy.get_param('~points')
        print ("target with", nb_points_vs, " points.")
    else:
        rospy.logwarn('no parameter given; using the default value %d' %nb_points_vs)
    
    pub_tracked_point = rospy.Publisher("tracked_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    pub_desired_point = rospy.Publisher("desired_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    subscriber()

