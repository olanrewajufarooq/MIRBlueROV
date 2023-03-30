
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import sys, os, glob
import datetime

import warnings

warnings.filterwarnings("ignore")

# Verify Input Argument
if (len(sys.argv) > 2):
	print ("invalid number of arguments:   " , str(len(sys.argv)))
	print ("should be 1: 'folder_path' ")
	sys.exit(1)
elif (len(sys.argv) == 2):
	print ("Plotting : ", sys.argv[1])
	folder_path = sys.argv[1]
else:
	print ("bad argument(s): ", str(sys.argv))	#shouldnt really come up
	sys.exit(1)

try: 
    plot_path = folder_path + "_plot"
    os.makedirs(plot_path)
except: #else already exists
    pass


# Checking the state of the BlueROV
mav_state_df = pd.read_csv(folder_path + "/_slash_br4_slash_mavros_slash_state.csv")
mav_state_df = mav_state_df[["rosbagTimestamp", "mode", "armed", "guided"]]
mav_state_df["rosbagTimestamp"] = mav_state_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))

# Checking when ARMED
mav_state_extract_df = mav_state_df[mav_state_df["armed"] == True]
timestamps = mav_state_extract_df['rosbagTimestamp']

# Extracting the start time and end time
start_time = timestamps.iloc[0]
end_time = timestamps.iloc[-1]


# EXTRACTING THE ORIENTATIONS
ang_deg_df = pd.read_csv(folder_path + "/_slash_br4_slash_angle_degree.csv")
ang_deg_df = ang_deg_df[["rosbagTimestamp", "x.1", "y.1", "z.1"]]
ang_deg_df.columns = ["rosbagTimestamp", "x", "y", "z"]
ang_deg_df["rosbagTimestamp"] = ang_deg_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


ang_deg_extract_df = ang_deg_df[ ang_deg_df["rosbagTimestamp"] >= start_time][ang_deg_df["rosbagTimestamp"] <= end_time]


# EXTRACTING THE ANGULAR RATES
ang_vel_df = pd.read_csv(folder_path + "/_slash_br4_slash_angular_velocity.csv")
ang_vel_df = ang_vel_df[["rosbagTimestamp", "x.1", "y.1", "z.1"]]
ang_vel_df.columns = ["rosbagTimestamp", "x", "y", "z"]
ang_vel_df["rosbagTimestamp"] = ang_vel_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


ang_vel_extract_df = ang_vel_df[ ang_vel_df["rosbagTimestamp"] >= start_time][ang_vel_df["rosbagTimestamp"] <= end_time]

# EXTRACTING THE THRUSTER VALUES
thrust_df = pd.read_csv(folder_path + "/_slash_br4_slash_mavros_slash_rc_slash_override.csv")
thrust_df["heave"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[2]))
thrust_df["yaw"] = thrust_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[3]))
thrust_df.drop("channels", axis = 1, inplace = True)
thrust_df["rosbagTimestamp"] = thrust_df["rosbagTimestamp"].apply(
    lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


thrust_extract_df = thrust_df[ thrust_df["rosbagTimestamp"] >= start_time][thrust_df["rosbagTimestamp"] <= end_time]

# EXTRACTING THE DEPTH VALUES

# depth_df = pd.read_csv(folder_path + "/_slash_br4_slash_mavros_slash_rc_slash_override.csv")
# depth_df["heave"] = depth_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[2]))
# depth_df["yaw"] = depth_df['channels'].apply(lambda x: int(x.strip("[]").split(",")[3]))
# depth_df.drop("channels", axis = 1, inplace = True)
# depth_df["rosbagTimestamp"] = depth_df["rosbagTimestamp"].apply(
#     lambda x: datetime.datetime.utcfromtimestamp(x // 1000000000) + datetime.timedelta(microseconds=x % 1000000000/1000))


# depth_extract_df = depth_df[ depth_df["rosbagTimestamp"] >= start_time][depth_df["rosbagTimestamp"] <= end_time]


# PLOTTING THE THRUSTER VALUES
plt.figure()

plt.plot( thrust_extract_df["rosbagTimestamp"], thrust_extract_df["heave"], '-r', thrust_extract_df["rosbagTimestamp"], thrust_extract_df["yaw"], '-g')
plt.ylim([1100, 1900])

plt.legend(["Heave", "Yaw"])
plt.title("Thruster PWM Plot")
plt.xlabel("Time")
plt.ylabel("PWM")

plt.savefig(plot_path + "/thruster_plot.png")


# PLOTTING THE ORIENTATION VALUES
plt.figure()

plt.plot( ang_deg_extract_df["rosbagTimestamp"], ang_deg_extract_df["z"], '-r')
plt.ylim([-180, 180])

plt.title("Yaw Angle Plot")
plt.xlabel("Time")
plt.ylabel("Angle (in Deg)")

plt.savefig(plot_path + "/yaw_plot.png")


# PLOTTING THE ANGULAR RATES VALUES
plt.figure()

plt.plot( ang_vel_extract_df["rosbagTimestamp"], ang_vel_extract_df["z"], '-r')

plt.title("Yaw Angular Rate Plot")
plt.xlabel("Time")
plt.ylabel("Angle (in Deg)")

plt.savefig(plot_path + "/yaw_rate_plot.png")


# PLOTTING THE DEPTH VALUES
# plt.figure()

# plt.plot( depth_extract_df["rosbagTimestamp"], depth_extract_df["z"], '-r')

# plt.title("Depth Plot")
# plt.xlabel("Time")
# plt.ylabel("Depth (in m)")

# plt.savefig(plot_path + "/yaw_rate_plot.png")


# PLOTTING EVERYTHING TOGETHER
fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(16, 10))

# plot something in each subplot
axs[0, 0].plot( thrust_extract_df["rosbagTimestamp"], thrust_extract_df["heave"], '-r', thrust_extract_df["rosbagTimestamp"], thrust_extract_df["yaw"], '-g')
axs[0, 0].set_ylim([1100, 1900])
axs[0, 0].legend(["Heave", "Yaw"])
axs[0, 0].set_title("Thruster PWM Plot")
axs[0, 0].set_xlabel("Time")
axs[0, 0].set_ylabel("PWM")

axs[0, 1].plot( ang_deg_extract_df["rosbagTimestamp"], ang_deg_extract_df["z"], '-r')
axs[0, 1].set_ylim([-180, 180])
axs[0, 1].set_title("Yaw Angle Plot")
axs[0, 1].set_xlabel("Time")
axs[0, 1].set_ylabel("Angle (in Deg)")

axs[1, 0].plot( ang_vel_df["rosbagTimestamp"], ang_vel_df["z"], '-r')
axs[1, 0].set_title("Yaw Angular Rate Plot")
axs[1, 0].set_xlabel("Time")
axs[1, 0].set_ylabel("Angle (in Deg)")

# axs[1, 1].plot( ang_vel_df["rosbagTimestamp"], ang_vel_df["z"], '-r')
# axs[1, 1].plot( depth_extract_df["rosbagTimestamp"], depth_extract_df["z"], '-r')
# axs[1, 1].title("Depth Plot")
# axs[1, 1].xlabel("Time")
# axs[1, 1].ylabel("Depth (in m)")


fig.suptitle('BlueROV Plots')
plt.tight_layout()

plt.savefig(plot_path + "/combined_plot.png")


