#!/usr/bin/env python

import rospy
import ast
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
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

    home_path = "/home/yifa/git/swiss_mile/navigation/rl_global_planner/log/eval_map1"
    files_path = ["trajectory_2024-9-7-8-33-9_LB.txt",
                  "trajectory_2024-9-7-8-43-43_far.txt"]  # Path to the text files
    color_list = [
                    [204, 0, 0],
                    [245, 121, 0],
    ]

    files_path = [os.path.join(home_path, file) for file in files_path]

    pubs = []
    for i in range(len(files_path)):
        pubs.append(rospy.Publisher(f'/trajectory_eval_{i}', Path, queue_size=10))
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        print("=================================")
        for i in range(len(files_path)):

            path = Path()

            points = read_points_from_file(files_path[i])
            # Set frame ID (e.g., "map" or "odom")
            path.header.frame_id = "map"

            for point in points:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                path.poses.append(pose)
            
            # point_cloud_msg = convert_to_point_cloud2(points, color_list[i])
            pubs[i].publish(path)
            rate.sleep()
            print(f"Published to /trajectory_{i}")

if __name__ == '__main__':
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass
