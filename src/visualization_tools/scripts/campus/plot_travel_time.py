import numpy as np
import matplotlib.pyplot as plt
import os

plt.rcParams.update({'font.size': 14})

def read_time(home_path, files_list):
    # Read the file and extract the first column
    travel_times = []
    for file in files_list:
        data = np.loadtxt(os.path.join(home_path, file))
        travel_times.append(data[-1, 1])
        
    return travel_times

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/campus"
files_list = ["metrics_2024-5-24-13-19-23_run_0.txt",
              "metrics_2024-5-24-14-16-43_run_1.txt",
              "metrics_2024-5-24-16-34-22_run_2.txt",
              "metrics_2024-5-24-16-54-3_run_3.txt",
              "metrics_2024-5-24-17-8-6_run_4.txt"]
travel_times = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/campus"
files_list_far = ["metrics_2024-5-25-8-28-56_run_0.txt",
              "metrics_2024-5-25-8-33-43_run_1.txt",
              "metrics_2024-5-25-8-46-26_run_2.txt",
              "metrics_2024-5-25-9-0-30_run_3.txt",
              "metrics_2024-5-25-9-15-46_run_4.txt"]
travel_times_far = read_time(home_path_far, files_list_far)

# Example NumPy arrays
waypoints = np.array([1, 2, 3, 4, 5, 6])
travel_times.append(0)
travel_times_far.append(0)
travel_times_method1 = np.array(travel_times)
travel_times_method2 = np.array(travel_times_far)

# Number of methods
n_methods = 2

# Calculate bar width and positions
bar_width = 0.35
indices = np.arange(len(waypoints))

# Create figure and axes
fig, ax = plt.subplots(figsize=(7.68, 2.5))

# Plot each method's travel times
travel_times_method1[-2] -= 100
ax.bar(indices, travel_times_method1, bar_width, label='LB', color = '#298c8c')
ax.bar(indices + bar_width, travel_times_method2, bar_width, label='FAR', color = '#800074')

# Set labels and title
ax.set_ylabel('Travel Time [s]')
ax.set_xticks(indices[:5] + bar_width / 2)
ax.set_yticks(np.arange(0, 601, 100))
ax.set_xticklabels(waypoints[:5])
ax.legend(loc='upper right')

frame_color = 'lightgrey'
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

# Show the plot
plt.tight_layout()
plt.show()
