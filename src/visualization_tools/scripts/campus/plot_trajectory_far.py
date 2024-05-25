#!/usr/bin/env python

import rospy
import ast
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import struct
import std_msgs.msg

def read_points_from_file(file_path):
    # Read the file and extract the relevant columns (x, y, z coordinates)
    data = np.loadtxt(file_path, usecols=(0, 1, 2))
    points = data.tolist()
    return points

def convert_to_point_cloud2(points, color):
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
    for point in points:
        x, y, z = point
        r, g, b = color[0], color[1], color[2]  # Change these values to set different colors
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
        point_cloud_data.append([x, y, z, rgb])

    point_cloud = point_cloud2.create_cloud(header, fields, point_cloud_data)
    return point_cloud

def point_cloud_publisher():
    rospy.init_node('point_cloud_publisher', anonymous=True)

    home_path = "/home/yifa/paper/autonomous_exploration_development_environment/src/vehicle_simulator/log/campus"
    files_path = ['trajectory_2024-5-25-8-28-56_run_0.txt',
                  "trajectory_2024-5-25-8-33-43_run_1.txt",
                  "trajectory_2024-5-25-8-46-26_run_2.txt",
                  "trajectory_2024-5-25-9-0-30_run_3.txt",
                  "trajectory_2024-5-25-9-15-46_run_4.txt"]  # Path to the text files
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
            points = read_points_from_file(files_path[i])
            point_cloud_msg = convert_to_point_cloud2(points, color_list[i])
            pubs[i].publish(point_cloud_msg)
            rate.sleep()
            print(f"Published to /trajectory_{i}")

if __name__ == '__main__':
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass
