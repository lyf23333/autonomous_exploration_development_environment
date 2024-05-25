import numpy as np
import matplotlib.pyplot as plt
import os

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

    

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/tunnel"
files_list = ["metrics_2024-5-22-21-33-45_run_0.txt",
              "metrics_2024-5-22-21-48-42_run_1.txt",
              "metrics_2024-5-22-22-14-55_run_2.txt",
              "metrics_2024-5-22-22-31-59_run_3.txt",
              "metrics_2024-5-22-23-22-45_run_4.txt",
              "metrics_2024-5-23-0-0-19_run_5.txt"]
total_data, total_run_time = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/tunnel"
files_list_far = ["metrics_2024-5-24-22-3-59_run_0.txt",
              "metrics_2024-5-24-22-8-34_run_1.txt",
              "metrics_2024-5-24-22-16-32_run_2.txt",
              "metrics_2024-5-24-22-24-30_run_3.txt",
              "metrics_2024-5-24-22-33-15_run_4.txt",
              "metrics_2024-5-24-22-37-40_run_5.txt"]
total_data_far, total_run_time_far = read_time(home_path_far, files_list_far)

# Plot the first column
plt.figure()
plt.plot(total_run_time, total_data, label='LBPlanner')
plt.plot(total_run_time_far, total_data_far, label='FARPlanner')
plt.xlabel('Time [s]')
plt.ylabel('Planning time [ms]')
plt.legend()

frame_color = 'lightgrey'
ax = plt.gca()
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

plt.grid(True)
plt.show()