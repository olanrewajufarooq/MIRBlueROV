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
t = 0
sample_time = 1/20

# Initialize Correction values
prev_depth_pred = 0
prev_depthrate_pred = 0

prev_yaw_pred = 0
prev_yawrate_pred = 0

Sum_Errors_dist = 0

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

	# if f > 0:
	# 	PWM = 1563 + 6.6610 * f
	# elif f < 0:
	# 	PWM = 1437 + 8.5565 * f
	if f > 0:
		PWM = 1536 + 9.3 * f
	elif f < 0:
		PWM = 1464 + 11.9 * f
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
	global Sum_Errors_dist

	global STATE
	global SEARCH_TYPE
	global USE_DEPTH_CONTROL

	global prev_depth_pred
	global prev_depthrate_pred
	global prev_yaw_pred
	global prev_yawrate_pred

	global Correction_depth
	
	global re_initialize_time
	global yaw_des
	global pause_for_state_change
	global stop_pause_time

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
		Sum_Errors_dist = 0

		STATE = "STOP"
		SEARCH_TYPE = "SWAY" # Options: "YAW", "SWAY"
		USE_DEPTH_CONTROL = True

		re_initialize_time = False
		yaw_des = 0

		pause_for_state_change = False
		stop_pause_time = 5

		prev_depth_pred = 0
		prev_depthrate_pred = 0
		prev_yaw_pred = 0
		prev_yawrate_pred = 0

		Correction_depth = 1500

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

	# Publihing the Distance Data
	
	current_dist = Float64()
	current_dist.data = pinger_distance
	pub_dist.publish(current_dist)


def OdoCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global init_a0
	global p
	global q
	global r
	global sample_time
	global t

	global Correction_depth

	global Sum_Errors_angle_yaw

	global pinger_distance
	global theta0
	global re_initialize_time
	global yaw_des

	global SEARCH_TYPE
	global STATE
	global USE_DEPTH_CONTROL

	global pause_for_state_change
	global stop_pause_time

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
		t = 0

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

	# PID CONTROL FOR HEADING

	# Initialize Sway Control
	Correction_sway = 1500

	# # STABILIZING YAW ANGLE
	if not (STATE == "SEARCH FREE PATH" and SEARCH_TYPE == "YAW"):
		# yaw_des = cubic_traj_yaw(t, theta0 = 0, dtheta = 0)
		yaw_des = 0
	
	# # SEARCHING FREE PATH
	# elif (STATE == "SEARCH FREE PATH") and (SEARCH_TYPE == "YAW"):
	# 	# Start time count
	# 	if re_initialize_time:
	# 		t = 0
	# 		theta0 = angle_wrt_startup[2]
	# 		re_initialize_time = False

	# 	yaw_des = cubic_traj_yaw(t, theta0 = theta0, dtheta = 90)
	# 	print(yaw_des)

	# # MOVING IN SWAY
	# elif (STATE == "SEARCH FREE PATH") and (SEARCH_TYPE == "SWAY"):
	# 	Correction_sway = 1550



	# yaw_des = 0 # desired yaw angle
	yaw_des_dot = 0 # desired yaw rate
	t = t + sample_time
	
	#PI parameters to tune
	# kp_yaw = 5e-8
	# ki_yaw = 1e-10
	# Kd_yaw = 1e-10

	kp_yaw = 0.005
	ki_yaw = 0.0001
	Kd_yaw = 0.001
	
	# error calculation
	yaw_err = yaw_des - angle_wrt_startup[2]
	Sum_Errors_angle_yaw += yaw_err * sample_time # we need to define SAMPLE TIME
	diff_error = yaw_des_dot - estimateRate(angle_wrt_startup[2], state="yaw")
	
	# PID controller 
	# f = -(kp_yaw * yaw_err + ki_yaw * Sum_Errors_angle_yaw)/4
	f = -(kp_yaw * yaw_err + ki_yaw * Sum_Errors_angle_yaw + Kd_yaw * diff_error)/4


	Correction_yaw = force_to_PWM(f) 
	Correction_yaw = int(Correction_yaw)


	# STATE MACHINE TRANSITIONS

	max_allowed_dist = 1000
	tol = 200
	
	if not pause_for_state_change:
		
		# pause_for_state_change = True
		# stop_pause_time = t + sample_time

		if STATE == "STOP" and (pinger_distance > max_allowed_dist):
			STATE = "SURGE"
		
		elif STATE == "STOP" and (pinger_distance <= max_allowed_dist):
			STATE = "SEARCH FREE PATH"
			re_initialize_time = True
		
		elif STATE == "SEARCH FREE PATH" and (pinger_distance >= max_allowed_dist):
		
			if SEARCH_TYPE == "YAW":
				init_a0 = True # Resetting current angle as the zero angle.
				yaw_des = 0
				Sum_Errors_angle_yaw = 0
			
			STATE = "SURGE"

		elif (STATE == "SURGE") and (pinger_distance < max_allowed_dist):
			STATE = "STOP"
	
	elif pause_for_state_change:
		if t >= stop_pause_time:
			pause_for_state_change = False


	# Printing State Information:

	print(f"STATE: {STATE:5}. Pinger Distance = {pinger_distance:5.2f}. Pinger Confidence = {pinger_confidence:3.2f}")
	


	# DISTANCE CONTROL

	if STATE in ["STOP", "SEARCH FREE PATH"]:
		Correction_dist = 1500
	
	elif STATE == "SURGE" and ( max_allowed_dist < pinger_distance < max_allowed_dist + tol ):
		Correction_dist = 1515
	
	elif STATE == "SURGE" and (pinger_distance > 8000):
		Correction_dist = 1640

	elif STATE == "SURGE" and (pinger_distance > 6000):
		Correction_dist = 1600

	elif STATE == "SURGE" and (pinger_distance > 4000):
		Correction_dist = 1560

	elif STATE == "SURGE":
		Correction_dist = 1530

	# SEARCH FREE PATH

	# Initialize Sway Control
	Correction_sway = 1500

	if (STATE == "SEARCH FREE PATH") and (SEARCH_TYPE == "YAW"):
	# 	# Start time count
	# 	if re_initialize_time:
	# 		t = 0
	# 		theta0 = angle_wrt_startup[2]
	# 		re_initialize_time = False

	# 	yaw_des = cubic_traj_yaw(t, theta0 = theta0, dtheta = 90)
	# 	print(yaw_des)
		
		yaw_des = yaw_des + 0.5


		if pinger_confidence < 60: 
			Correction_yaw = 1500
			Correction_dist = 1440


		if yaw_des >= 360:
			yaw_des = 0

	# # MOVING IN SWAY
	elif (STATE == "SEARCH FREE PATH") and (SEARCH_TYPE == "SWAY"):
		Correction_sway = 1550

	

	if USE_DEPTH_CONTROL:
		setOverrideRCIN(1500, 1500, Correction_depth, Correction_yaw, Correction_dist, Correction_sway)
	else:
		setOverrideRCIN(1500, 1500, 1500, Correction_yaw, Correction_dist, Correction_sway)


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


def PressureCallback(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	
	global t
	global sample_time
	global prev_depth
	
	global Sum_Errors_depth

	global Correction_depth

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
	
	depth_des = 0.2
	depth_des_dot = 0
	# depth_des, depth_des_dot = cubic_traj_depth(t) # For Practical Work 2

	t = t + sample_time
	
	Kp_depth = 50
	Ki_depth = 0.015
	Kd_depth = 0

	floatability = 14

	error = depth_des - depth_wrt_startup
	Sum_Errors_depth += error * sample_time
	diff_error = depth_des_dot - estimateRate(depth_wrt_startup, state="depth")
	
	# Updating the value of prev_depth for depth rate estimation
	prev_depth = depth_wrt_startup 

	# f = -(Kp_depth * error + Ki_depth * Sum_Errors_depth + floatability)/4
	f = -( Kp_depth * error + Ki_depth * Sum_Errors_depth + Kd_depth * diff_error + floatability )/4

	# update Correction_depth
	Correction_depth = force_to_PWM(f)
	Correction_depth = int(Correction_depth)

	# setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)


def cubic_traj_depth(current_time):

	z_init = 0
	z_final = 0.4
	t_final = 20

	if current_time < t_final:
		a2 = 3 * (z_final - z_init) / (t_final ** 2)
		a3 = -2 * (z_final - z_init) / (t_final ** 3)

		z = z_init + a2 * current_time**2 + a3 * current_time**3
		z_dot = 2 * a2*current_time + 3 * a3 * current_time**2

	elif current_time >= t_final:
		z = z_final
		z_dot = 0

	return z, z_dot


def cubic_traj_yaw(current_time, theta0 = 0, dtheta = 90):

	z_init = theta0
	z_final = theta0 + dtheta
	t_final = 10

	if current_time < t_final:
		a2 = 3 * (z_final - z_init) / (t_final ** 2)
		a3 = -2 * (z_final - z_init) / (t_final ** 3)

		z = z_init + a2 * current_time**2 + a3 * current_time**3

	elif current_time >= t_final:
		z = z_final

	return z


def estimateRate(new_measurement, state = "depth", alpha = 0.45, beta = 0.1):
	global sample_time

	global prev_depth_pred
	global prev_depthrate_pred
	
	global prev_yaw_pred
	global prev_yawrate_pred

	if state == "depth":
		prev_state_pred = prev_depth_pred
		prev_staterate_pred = prev_depthrate_pred
	elif state == "yaw":
		prev_state_pred = prev_yaw_pred
		prev_staterate_pred = prev_yawrate_pred

    # Predicting current states
	state_pred = prev_state_pred + sample_time * prev_staterate_pred
	staterate_pred = prev_staterate_pred

	# Computing the errors
	error = new_measurement - state_pred

	# Computing new state estimates
	updated_state_pred = state_pred + alpha * error
	updated_staterate_pred = staterate_pred + (beta / sample_time) * error

	if state == "depth":
		prev_state_pred = updated_state_pred
		prev_staterate_pred = updated_staterate_pred
	elif state == "yaw":
		prev_state_pred = updated_state_pred
		prev_staterate_pred = updated_staterate_pred

	return updated_staterate_pred



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

	pub_dist = rospy.Publisher('robot_distance', Float64, queue_size = 10, tcp_nodelay = True)

	pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

	subscriber()

