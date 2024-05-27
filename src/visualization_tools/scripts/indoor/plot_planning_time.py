import numpy as np
import matplotlib.pyplot as plt
import os
plt.rcParams.update({'font.size': 16})

def read_time(home_path, files_list):
    # Read the file and extract the first column
    total_data = None
    total_run_time = None
    for file in files_list:
        data = np.loadtxt(os.path.join(home_path, file))
        planning_time = data[:, 3]
        run_time = data[:,4]
        if total_data is None:
            total_data = planning_time
            total_run_time = run_time
        else:
            total_data = np.hstack((total_data, planning_time))
            total_run_time = np.hstack((total_run_time, run_time + np.max(total_run_time)))
    return total_data, total_run_time

    

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/indoor"
files_list = ["metrics_2024-5-20-23-24-13_run_0.txt",
              "metrics_2024-5-21-21-29-38_run_1.txt",
              "metrics_2024-5-21-21-41-38_run_2.txt",
              "metrics_2024-5-21-21-52-20_run_3.txt",
              "metrics_2024-5-21-21-59-11_run_4.txt",
              "metrics_2024-5-21-22-24-8_run_5.txt"]
total_data, total_run_time = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/indoor"
files_list_far = ["metrics_2024-5-21-23-4-24_run_0.txt",
              "metrics_2024-5-21-23-19-25_run_1.txt",
              "metrics_2024-5-22-8-36-43_run_2.txt",
              "metrics_2024-5-22-8-42-13_run_3.txt",
              "metrics_2024-5-22-8-49-34_run_4.txt",
              "metrics_2024-5-22-20-56-16_run_5.txt"]
total_data_far, total_run_time_far = read_time(home_path_far, files_list_far)

# Plot the first column
fig = plt.figure(figsize=(7.68, 2.5))
ax = plt.subplot()
ax.plot(total_run_time, total_data, label='LB', color = '#a00000')
ax.plot(total_run_time_far, total_data_far, label='FAR', color = '#1a80bb')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Planning time [ms]')
plt.tight_layout()  # Adjusts the layout

frame_color = 'lightgrey'
ax = plt.gca()
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

plt.grid(True)
plt.show()