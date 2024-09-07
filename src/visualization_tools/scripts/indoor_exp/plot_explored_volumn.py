import numpy as np
import matplotlib.pyplot as plt
import os
plt.rcParams.update({'font.size': 14})

def read_time(home_path, files_list):
    # Read the file and extract the first column
    total_data = None
    total_run_time = None
    for file in files_list:
        data = np.loadtxt(os.path.join(home_path, file))
        planning_time = data[:, 0]
        run_time = data[:,4]
        if total_data is None:
            total_data = planning_time
            total_run_time = run_time
        else:
            total_data = np.hstack((total_data, planning_time))
            total_run_time = np.hstack((total_run_time, run_time + np.max(total_run_time)))
    return total_data, total_run_time

    

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/indoor_exp"
files_list = ["metrics_2024-6-1-23-0-21_run0.txt"]
total_data, total_run_time = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/indoor_exp"
files_list_far = ["metrics_2024-6-2-0-16-4_indoor.txt"]
total_data_far, total_run_time_far = read_time(home_path_far, files_list_far)

# Plot the first column
fig = plt.figure(figsize=(7.68, 2.8))
ax = plt.subplot()
ax.plot(total_run_time, 0.27 *total_data, label='LBE', color = '#a00000')
ax.plot(total_run_time_far, total_data_far, label='TARE', color = '#1a80bb')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Explored Volume [m$^3$]')
plt.tight_layout()  # Adjusts the layout

# Set y-axis range and ticks
ax.set_ylim(0, 6000)
ax.set_yticks(range(0, 6001, 2000))

frame_color = 'lightgrey'
ax = plt.gca()
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

plt.legend()

plt.show()