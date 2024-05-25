import numpy as np
import matplotlib.pyplot as plt
import os

def read_time(home_path, files_list):
    # Read the file and extract the first column
    travel_times = []
    for file in files_list:
        data = np.loadtxt(os.path.join(home_path, file))
        travel_times.append(data[-1, -1])
        
    return travel_times

home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/tunnel"
files_list = ["metrics_2024-5-22-21-33-45_run_0.txt",
              "metrics_2024-5-22-21-48-42_run_1.txt",
              "metrics_2024-5-22-22-14-55_run_2.txt",
              "metrics_2024-5-22-22-31-59_run_3.txt",
              "metrics_2024-5-22-23-22-45_run_4.txt",
              "metrics_2024-5-23-0-0-19_run_5.txt"]
travel_times = read_time(home_path, files_list)

home_path_far = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/tunnel"
files_list_far = ["metrics_2024-5-24-22-3-59_run_0.txt",
              "metrics_2024-5-24-22-8-34_run_1.txt",
              "metrics_2024-5-24-22-16-32_run_2.txt",
              "metrics_2024-5-24-22-24-30_run_3.txt",
              "metrics_2024-5-24-22-33-15_run_4.txt",
              "metrics_2024-5-24-22-37-40_run_5.txt"]
travel_times_far = read_time(home_path_far, files_list_far)

# Example NumPy arrays
waypoints = np.array([1, 2, 3, 4, 5, 6])
travel_times_method1 = np.array(travel_times)
travel_times_method2 = np.array(travel_times_far)

# Number of methods
n_methods = 2

# Calculate bar width and positions
bar_width = 0.35
indices = np.arange(len(waypoints))

# Create figure and axes
fig, ax = plt.subplots(figsize=(12, 5))

# Plot each method's travel times
ax.bar(indices, travel_times_method1, bar_width, label='LBPlanner', color = '#a00000')
ax.bar(indices + bar_width, travel_times_method2, bar_width, label='FARPlanner', color = '#1a80bb')

# Set labels and title
ax.set_xlabel('Waypoint')
ax.set_ylabel('Travel Time [s]')
ax.set_xticks(indices + bar_width / 2)
ax.set_yticks(np.arange(0, 501, 100))
ax.set_xticklabels(waypoints)
ax.legend()

frame_color = 'lightgrey'
for spine in ax.spines.values():
    spine.set_edgecolor(frame_color)

# Show the plot
plt.tight_layout()
plt.show()
