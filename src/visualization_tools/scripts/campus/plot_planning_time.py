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
        planning_time = data[:, 3]
        run_time = data[:,4]
        if total_data is None:
            total_data = planning_time
            total_run_time = run_time
        else:
            total_data = np.hstack((total_data, planning_time))
            total_run_time = np.hstack((total_run_time, run_time + np.max(total_run_time)))
    return total_data, total_run_time

    

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/campus"
files_list = ["metrics_2024-5-24-13-19-23_run_0.txt",
              "metrics_2024-5-24-14-16-43_run_1.txt",
              "metrics_2024-5-24-16-34-22_run_2.txt",
              "metrics_2024-5-24-16-54-3_run_3.txt",
              "metrics_2024-5-24-17-8-6_run_4.txt"]
total_data, total_run_time = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/campus"
files_list_far = ["metrics_2024-5-25-8-28-56_run_0.txt",
              "metrics_2024-5-25-8-33-43_run_1.txt",
              "metrics_2024-5-25-8-46-26_run_2.txt",
              "metrics_2024-5-25-9-0-30_run_3.txt",
              "metrics_2024-5-25-9-15-46_run_4.txt"]
total_data_far, total_run_time_far = read_time(home_path_far, files_list_far)

# Plot the first column
fig = plt.figure(figsize=(7.68, 2.5))
ax = plt.subplot()
ax.plot(total_run_time, total_data, label='LB', color = '#298c8c')
ax.plot(total_run_time_far, total_data_far, label='FAR', color = '#800074')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Planning time [ms]')
plt.tight_layout()  # Adjusts the layout

frame_color = 'lightgrey'
ax = plt.gca()
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

plt.grid(True)
plt.show()