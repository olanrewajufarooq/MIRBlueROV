
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import sys, os, glob
import datetime
import shutil

import warnings

warnings.filterwarnings("ignore")

# Verify Input Argument
if (len(sys.argv) > 3):
	print ("invalid number of arguments:   " , str(len(sys.argv)))
	print ("should be 2: 'folder_path' 'depth_control' ")
	sys.exit(1)
elif (len(sys.argv) == 3):
	print ("Plotting : ", sys.argv[1])
	folder_path = sys.argv[1]
	depth_control = bool(sys.argv[2])
elif (len(sys.argv) == 2):
	print ("Plotting : ", sys.argv[1])
	folder_path = sys.argv[1]
	depth_control = False
else:
	print ("bad argument(s): ", str(sys.argv))	#shouldnt really come up
	sys.exit(1)

try: 
    plot_path = folder_path + "_plot"
    os.makedirs(plot_path)
except: #else already exists
    pass


mav_state_df = pd.read_csv(folder_path + "/_slash_br4_slash_mavros_slash_state.csv")
mav_state_df = mav_state_df[["rosbagTimestamp", "mode", "armed", "guided"]]
mav_state_df["rosbagTimestamp"] = mav_state_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


mav_state_extract_df = mav_state_df[mav_state_df["armed"] == True]
timestamps = mav_state_extract_df['rosbagTimestamp']

start_time = timestamps.iloc[0]
end_time = timestamps.iloc[-1]


depth_df = pd.read_csv(folder_path + "/_slash_br4_slash_depth_slash_state.csv")
depth_df.columns = ["rosbagTimestamp", "depth"]
depth_df["rosbagTimestamp"] = depth_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


depth_extract_df = depth_df[ depth_df["rosbagTimestamp"] >= start_time][depth_df["rosbagTimestamp"] <= end_time]


dist_df = pd.read_csv(folder_path + "/_slash_br4_slash_robot_distance.csv")
dist_df.columns = ["rosbagTimestamp", "dist"]
dist_df["dist"] = dist_df["dist"]/1000 # Converting to meters
dist_df["rosbagTimestamp"] = dist_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


dist_extract_df = dist_df[ dist_df["rosbagTimestamp"] >= start_time][dist_df["rosbagTimestamp"] <= end_time]


ang_deg_df = pd.read_csv(folder_path + "/_slash_br4_slash_angle_degree.csv")
ang_deg_df = ang_deg_df[["rosbagTimestamp", "x.1", "y.1", "z.1"]]
ang_deg_df.columns = ["rosbagTimestamp", "x", "y", "z"]
ang_deg_df["rosbagTimestamp"] = ang_deg_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


ang_deg_extract_df = ang_deg_df[ ang_deg_df["rosbagTimestamp"] >= start_time][ang_deg_df["rosbagTimestamp"] <= end_time]


ang_vel_df = pd.read_csv(folder_path + "/_slash_br4_slash_angular_velocity.csv")
ang_vel_df = ang_vel_df[["rosbagTimestamp", "x.1", "y.1", "z.1"]]
ang_vel_df.columns = ["rosbagTimestamp", "x", "y", "z"]
ang_vel_df["rosbagTimestamp"] = ang_vel_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


ang_vel_extract_df = ang_vel_df[ ang_vel_df["rosbagTimestamp"] >= start_time][ang_vel_df["rosbagTimestamp"] <= end_time]


thrust_df = pd.read_csv(folder_path + "/_slash_br4_slash_mavros_slash_rc_slash_override.csv")
thrust_df["heave"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[2]))
thrust_df["yaw"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[3]))
thrust_df["surge"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[4]))
thrust_df["sway"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[5]))
thrust_df.drop("channels", axis = 1, inplace = True)
thrust_df["rosbagTimestamp"] = thrust_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


thrust_extract_df = thrust_df[ thrust_df["rosbagTimestamp"] >= start_time][thrust_df["rosbagTimestamp"] <= end_time]


# PLOTTING THRUSTS (YAW, HEAVE, SURGE)
plt.figure()

plt.plot( thrust_extract_df["rosbagTimestamp"], thrust_extract_df["yaw"], '-g', thrust_extract_df["rosbagTimestamp"], thrust_extract_df["heave"], '-r',
         thrust_extract_df["rosbagTimestamp"], thrust_extract_df["surge"], '-b')
plt.ylim([ 1400, 1600 ])

# desired_ticks = 6
# plt.gca().xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

plt.legend(["Yaw", "Heave", "Surge"])
plt.title("Thruster PWM Plot")
plt.xlabel("Time")
plt.ylabel("PWM")

plt.savefig(plot_path + "/thruster_plot_with_yaw.png")


# PLOTTING THRUSTS (SWAY, HEAVE, SURGE)
plt.figure()

plt.plot( thrust_extract_df["rosbagTimestamp"], thrust_extract_df["sway"], '-g', thrust_extract_df["rosbagTimestamp"], thrust_extract_df["heave"], '-r',
         thrust_extract_df["rosbagTimestamp"], thrust_extract_df["surge"], '-b')
plt.ylim([ 1400, 1600 ])

# desired_ticks = 6
# plt.gca().xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

plt.legend(["Sway", "Heave", "Surge"])
plt.title("Thruster PWM Plot")
plt.xlabel("Time")
plt.ylabel("PWM")

plt.savefig(plot_path + "/thruster_plot_with_sway.png")


# PLOTTING YAW ANGLE, DISTANCE AND DEPTH
plt.figure()

ax1 = plt.subplot()
plot1, = ax1.plot(dist_extract_df["rosbagTimestamp"], dist_extract_df["dist"], '-b')
plot2, = ax1.plot(depth_extract_df["rosbagTimestamp"], depth_extract_df["depth"], '-g')
ax1.set_ylabel("Positions (in m)")
ax1.grid()

ax2 = ax1.twinx()
plot3, = ax2.plot( ang_deg_extract_df["rosbagTimestamp"], ang_deg_extract_df["z"], '-r')
ax2.set_ylim([-180, 180])
ax2.set_ylabel("Angles (in deg)")

# desired_ticks = 6
# ax1.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))
# ax2.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

plt.legend([plot1, plot2, plot3], ["Distance (m)", "Depth (m)", "Angles (deg)"])

plt.title("State Plot")
plt.xlabel("Time")

plt.savefig(plot_path + "/states.png")


# PLOTTING YAW AND SURGE AGAINST DISTANCE
plt.figure()

ax1 = plt.subplot()
plot1, = ax1.plot(thrust_extract_df["rosbagTimestamp"], thrust_extract_df["yaw"], '-y')
plot2, = ax1.plot(thrust_extract_df["rosbagTimestamp"], thrust_extract_df["surge"], '-r')
ax1.set_ylim([1400, 1600])
ax1.set_ylabel("Thrusts (in PWM)")
ax1.grid()

ax2 = ax1.twinx()
plot3, = ax2.plot(dist_extract_df["rosbagTimestamp"], dist_extract_df["dist"], '-b')
ax2.set_ylabel("Positions (in m)")
ax2.grid(axis='x')

# desired_ticks = 6
# ax1.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))
# ax2.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

plt.legend([plot1, plot2, plot3], ["Yaw Thrust (in PWM)", "Surge Thrust (in PWM)", "Distance (m)"])
plt.title("Free Path Search with Yaw ")
plt.xlabel("Time")

plt.savefig(plot_path + "/free_path_yaw.png")


# PLOTTING SWAY AND SURGE AGAINST DISTANCE
plt.figure()

ax1 = plt.subplot()
plot1, = ax1.plot(thrust_extract_df["rosbagTimestamp"], thrust_extract_df["sway"], '-y')
plot2, = ax1.plot(thrust_extract_df["rosbagTimestamp"], thrust_extract_df["surge"], '-r')
ax1.set_ylim([1400, 1600])
ax1.set_ylabel("Thrusts (in PWM)")
ax1.grid()

ax2 = ax1.twinx()
plot3, = ax2.plot(dist_extract_df["rosbagTimestamp"], dist_extract_df["dist"], '-b')
ax2.set_ylabel("Positions (in m)")
ax2.grid(axis='x')

# desired_ticks = 6
# ax1.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))
# ax2.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

plt.legend([plot1, plot2, plot3], ["Sway Thrust (in PWM)", "Surge Thrust (in PWM)", "Distance (m)"])
plt.title("Free Path Search with Yaw ")
plt.xlabel("Time")

plt.savefig(plot_path + "/free_path_sway.png")


# PLOTTING DEPTH AGAINST HEAVE CONTROL
if depth_control:
	plt.figure()
	
	ax1 = plt.subplot()
	plot1, = ax1.plot(thrust_extract_df["rosbagTimestamp"], thrust_extract_df["heave"], '-r')
	ax1.set_ylim([1400, 1600])
	ax1.set_ylabel("Heave Thrusts (in PWM)")
	ax1.grid()

	ax2 = ax1.twinx()
	plot2, = ax2.plot(depth_extract_df["rosbagTimestamp"], depth_extract_df["depth"], '-b')
	ax2.set_ylabel("Depth (in m)")
	ax2.grid(axis='x')

	# desired_ticks = 6
	# ax1.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))
	# ax2.xaxis.set_major_locator(plt.MaxNLocator(desired_ticks))

	plt.legend([plot1, plot2], ["Heave Thrust (in PWM)", "Depth (m)"])

	plt.title("Plot of Depth and Heave")
	plt.xlabel("Time")

	plt.savefig(plot_path + "/depth.png")


shutil.rmtree(folder_path) # Delete the extracted csv files (To free up space)
os.remove(f"{folder_path}.bag") # Delete the bag that was filtered (To remove excessive files)