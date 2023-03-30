#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken

import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
#from waterlinked_a50_ros_driver.msg import DVL
#from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
import time
import sys
import argparse

# ---------- Global Variables ---------------

set_mode = [0]*3
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

#Conditions
init_a0 = True
init_p0 = True
arming = False

angle_wrt_startup = [0]*3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0
depth_wrt_startup = 0
depth_p0 = 0

enable_depth = False 
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

Vmax_mot = 1900
Vmin_mot = 1100

#Sample time
sample_time = 1/20

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# ---------- Functions---------------

# TASK 1.1 Start
def force_to_PWM(f):
	"""
	This is a function that converts the force values to PWM for the motors.

	16V:
	Positive - Intercept: 1563.8915056025264, Slope: 6.6609554433778975
	Negative - Intercept: 1437.5105488646336, Slope: 8.556474591368337

	14.8V:
	Positive - Intercept: 1563.8915056025264, Slope: 6.6609554433778975
	Negative - Intercept: 1437.5105488646336, Slope: 8.556474591368337
	
	11.1V:
	Positive - Intercept: 1563.8915056025264, Slope: 6.6609554433778975
	Negative - Intercept: 1437.5105488646336, Slope: 8.556474591368337

	12V:
	Positive - Intercept: 1568.1841920414354, Slope: 9.264095453590867
	Negative - Intercept: 1433.6772068472728, Slope: 11.876213274212262

	Currently in Use: 16V
	"""

	if f > 0:
		PWM = 1536 + 9.2641 * f
	elif f < 0:
		PWM = 1464 + 11.8762 * f
	else:
		PWM = 1500
	
	return PWM
# TASK 1.1 END


def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global Sum_Errors_Vel
	global Sum_Errors_angle_yaw
	global Sum_Errors_depth

	# Joystick buttons
	btn_arm = data.buttons[7]  # Start button
	btn_disarm = data.buttons[6]  # Back button
	btn_manual_mode = data.buttons[3]  # Y button
	btn_automatic_mode = data.buttons[2]  # X button
	btn_corrected_mode = data.buttons[0]  # A button

	# Disarming when Back button is pressed
	if (btn_disarm == 1 and arming == True):
		arming = False
		armDisarm(arming)

	# Arming when Start button is pressed
	if (btn_arm == 1 and arming == False):
		arming = True
		armDisarm(arming)

	# Switch manual, auto and correction mode
	if (btn_manual_mode and not set_mode[0]):
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False		
		rospy.loginfo("Mode manual")
	if (btn_automatic_mode and not set_mode[1]):
		set_mode[0] = False
		set_mode[1] = True
		set_mode[2] = False		
		rospy.loginfo("Mode automatic")
	if (btn_corrected_mode and not set_mode[2]):
		init_a0 = True
		init_p0 = True
		# set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3

		# TASK [All PI and PID Controls]
		Sum_Errors_depth = 0
		Sum_Errors_angle_yaw = 0
		# TASK End

		set_mode[0] = False
		set_mode[1] = False
		set_mode[2] = True
		rospy.loginfo("Mode correction")


def armDisarm(armed):
	# This functions sends a long command service with 400 code to arm or disarm motors
	if (armed):
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Arming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except arming")
	else:
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except disarming")	


def velCallback(cmd_vel):
	global set_mode

	# Only continue if manual_mode is enabled
	if (set_mode[1] or set_mode[2]):
		return

	# Extract cmd_vel message
	roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)


def pingerCallback(data):
	global pinger_confidence
	global pinger_distance

	pinger_distance = data.data[0]
	pinger_confidence = data.data[1]


def OdoCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global init_a0
	global p
	global q
	global r

	global Sum_Errors_angle_yaw

	orientation = data.orientation
	angular_velocity = data.angular_velocity

	# extraction of yaw angle
	quarternion = [orientation.x, orientation.y, orientation.z, orientation.w]
	euler = tf.transformations.euler_from_quaternion(quarternion)
	angle_roll = euler[0]
	angle_pitch = euler[1]
	angle_yaw = euler[2]

	if (init_a0):
		# at 1st execution, init
		angle_roll_a0 = angle_roll
		angle_pitch_a0 = angle_pitch
		angle_yaw_a0 = angle_yaw
		init_a0 = False

	angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	
	angle = Twist()
	angle.angular.x = angle_wrt_startup[0]
	angle.angular.y = angle_wrt_startup[1]
	angle.angular.z = angle_wrt_startup[2]

	pub_angle_degre.publish(angle)

	# Extraction of angular velocity
	p = angular_velocity.x
	q = angular_velocity.y
	r = angular_velocity.z

	vel = Twist()
	vel.angular.x = p
	vel.angular.y = q
	vel.angular.z = r

	pub_angular_velocity.publish(vel)

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return

	# Send PWM commands to motors
	# yaw command to be adapted using sensor feedback

	# TASK 1.4, 2.5 Start
	
	yaw_des = 0 # desired yaw angle
	
	#PI parameters to tune  
	kp_yaw = 0.01
	ki_yaw = 0
	
	# error calculation
	yaw_err = yaw_des - angle_wrt_startup[2]
	Sum_Errors_angle_yaw += yaw_err * sample_time # we need to define SAMPLE TIME
	
	# PI controller 
	control_yaw = kp_yaw * yaw_err + ki_yaw * Sum_Errors_angle_yaw
	Correction_yaw = force_to_PWM (control_yaw) 
	
	# TASK 1.4, 2.5 End

	setOverrideRCIN(1500, 1500, 1500, Correction_yaw, 1500, 1500)


def DvlCallback(data):
	global set_mode
	global u
	global v
	global w

	u = data.velocity.x  # Linear surge velocity
	v = data.velocity.y  # Linear sway velocity
	w = data.velocity.z  # Linear heave velocity

	Vel = Twist()
	Vel.linear.x = u
	Vel.linear.y = v
	Vel.linear.z = w
	pub_linear_velocity.publish(Vel)


def cubic_traj(t):

	z_init = 0
	z_final = 0.8
	t_final = 20

	if t < t_final:
		a2 = 3 * (z_final - z_init) / (t ** 2)
		a3 = -2 * (z_final - z_init) / (t ** 3)

		z = z_init + a2 * t**2 + a3 * t**3
		z_dot = 2 * a2*t + 3 * a3 * t**2

	elif t >= t_final:
		z = z_final
		z_dot = 0

	return z, z_dot

# TASK 2.6 Start
def estimateHeave(depth, prev_depth=None, prev_heave=None):
	global sample_time

	alpha = 0.45
	beta = 0.1

	if prev_depth is None:
		heave = 0
	else:
		heave = (depth - prev_depth) / sample_time

    # Update position and velocity estimates using alpha-beta filter
	if prev_heave is None:
		filtered_depth = depth
		filtered_heave = heave
		
	else:
		filtered_depth = prev_depth + sample_time * prev_heave + 0.5 * beta * sample_time**2 * (heave + prev_heave)
		filtered_heave = prev_heave + alpha * sample_time * (heave + prev_heave)

	return filtered_heave
# TASK 2.6 End


def PressureCallback(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	global t
	
	global Sum_Errors_depth

	rho = 1000.0 # 1025.0 for sea water
	g = 9.80665

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return
	elif (set_mode[1]):
		# Only continue if automatic_mode is enabled
		# Define an arbitrary velocity command and observe robot's velocity
		setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
		return

	pressure = data.fluid_pressure

	if (init_p0):
		# 1st execution, init
		depth_p0 = (pressure - 101300)/(rho*g)
		t = 0
		init_p0 = False

	depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0

	# Publishing the Depth Data
	
	current_depth = Float64()
	current_depth.data = depth_wrt_startup
	pub_depth.publish(current_depth)

	# setup depth servo control here
	# TASK 1.5, 1.6, 2.3, 2.5, 2.7 Start
	depth_des = 0.8
	
	#depth_des, depth_des_dot = cubic_traj(t) # For Practical Work 2
	#t = t + sample_time
	
	Kp_depth = 0
	Ki_depth = 0
	Kd_depth = 0

	floatability = 3.5 # Adding the floatability as a PMW value.

	error = depth_des - depth_wrt_startup
	Sum_Errors_depth += error * sample_time

	f = Kp_depth * error + Ki_depth * Sum_Errors_depth + floatability 
	#f = Kp_depth * error + Ki_depth * Sum_Errors_depth + Kd_depth * ( depth_des_dot - estimateHeave(depth_wrt_startup) ) + floatability 

	# update Correction_depth
	Correction_depth = force_to_PWM(f)
	# Correction_depth = int(1595.9765)

	# TASK 1.5, 1.6, 2.3, 2.5, 2.7 End

	# Send PWM commands to motors
	Correction_depth = int(Correction_depth)
	setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)


def mapValueScalSat(value):
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 400 + 1500

	# Saturation
	if pulse_width > 1900:
		pulse_width = 1900
	if pulse_width < 1100:
		pulse_width = 1100

	return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	msg_override = OverrideRCIn()
	msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
	msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
	msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
	msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500

	pub_msg_override.publish(msg_override)


def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	#rospy.Subscriber("/dvl/data", DVL, DvlCallback)
	rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
	rospy.spin()


if __name__ == '__main__':
	armDisarm(False)  # Not automatically disarmed at startup
	rospy.init_node('autonomous_MIR', anonymous=False)
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
	pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)

	pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

	subscriber()

