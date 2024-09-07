#!/usr/bin/env python

import rospy
import ast
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import struct
import std_msgs.msg
import matplotlib.cm as cm


def read_time(home_path, files_list):
    # Read the file and extract the first column
    total_data = None
    total_run_time = None
    for file in files_list:
        data = np.loadtxt(os.path.join(home_path, file))
        planning_time = data[:, 1]
        run_time = data[:,4]
        if total_data is None:
            total_data = planning_time
            total_run_time = run_time
        else:
            total_data = np.hstack((total_data, planning_time))
            total_run_time = np.hstack((total_run_time, run_time + np.max(total_run_time)))
    return total_data, total_run_time

def intensity_to_rgb(intensity, cmap_name='viridis'):
    """
    Convert a scalar intensity value to an RGB color using a colormap.
    :param intensity: Scalar intensity value (e.g., between 0 and 1).
    :param cmap_name: Name of the matplotlib colormap to use.
    :return: (r, g, b) tuple with values in range 0-255.
    """
    cmap = cm.get_cmap(cmap_name)
    normalized_intensity = intensity
    rgba = cmap(normalized_intensity)
    r, g, b, _ = [int(255 * c) for c in rgba]
    return r, g, b

def read_points_from_file(file_path):
    # Read the file and extract the relevant columns (x, y, z coordinates)
    data = np.loadtxt(file_path, usecols=(0, 1, 2))
    points = data.tolist()
    return points

def convert_to_point_cloud2(points, color, traveling_distance):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.FLOAT32, 1)
    ]

    point_cloud_data = []
    for i in range(len(points)):
        x, y, z = points[i]
        point_cloud_data.append([x, y, z, traveling_distance[i]/max(traveling_distance)])

    point_cloud = point_cloud2.create_cloud(header, fields, point_cloud_data)
    return point_cloud

def point_cloud_publisher():
    rospy.init_node('point_cloud_publisher', anonymous=True)

    home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/indoor_exp"
    files_path = ['trajectory_2024-6-1-23-0-21_run0.txt']  # Path to the text files
    color_list = [
                    [204, 0, 0],
                    [245, 121, 0],
                    [237, 212, 0],
                    [115, 210, 22],
                    [115, 210, 22],
                    [115, 210, 22]
    ]

    files_path = [os.path.join(home_path, file) for file in files_path]

    pubs = []
    for i in range(len(files_path)):
        pubs.append(rospy.Publisher(f'/trajectory_{i}', PointCloud2, queue_size=10))
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        print("=================================")
        for i in range(len(files_path)):
            home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/indoor_exp"
            files_list = ["metrics_2024-6-1-23-0-21_run0.txt"]
            traveling_distance, total_run_time = read_time(home_path, files_list)

            points = read_points_from_file(files_path[i])
            point_cloud_msg = convert_to_point_cloud2(points, color_list[i], traveling_distance)
            pubs[i].publish(point_cloud_msg)
            rate.sleep()
            print(f"Published to /trajectory_{i}")

if __name__ == '__main__':
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass
