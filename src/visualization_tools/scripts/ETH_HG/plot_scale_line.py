#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_text_marker(text, x, y, z=10, marker_id=0):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "scale_marker"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.z = z  # Height of the text
    marker.color.a = 1.0  # Transparency
    marker.color.r = 0.8  # Red
    marker.color.g = 0.2  # Green
    marker.color.b = 0.0  # Blue
    marker.text = text

    return marker

def create_text_marker_planner(text, x, y, z=10, marker_id=0):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "scale_marker"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.z = z  # Height of the text
    marker.color.a = 1.0  # Transparency
    marker.color.r = 1.0 # Red
    marker.color.g = 1.0 # Green
    marker.color.b = 1.0  # Blue
    marker.text = text

    return marker

def create_line_marker(x_start, y_start, x_end, y_end, z, marker_id, scale_x):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "scale_marker"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale_x  # Line width
    marker.color.a = 1.0  # Transparency
    marker.color.r = 0.8  # Red
    marker.color.g = 0.2  # Green
    marker.color.b = 0.0  # Blue

    marker.points = [
        Point(x_start, y_start, z),
        Point(x_end, y_end, z)
    ]

    return marker

def scale_visualization_publisher():
    rospy.init_node('scale_visualization_publisher', anonymous=True)
    pub = rospy.Publisher('/scale_line', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    marker_array = MarkerArray()

    scale_length=30
    x_start = -20
    y_start = 60
    line_width = 1.0

    # # Text Marker
    # text = f"{scale_length}m"
    # text_marker = create_text_marker(text, x_start + scale_length/2 + 5, y_start + 5, 10, 0)
    # marker_array.markers.append(text_marker)

    # # Text Marker
    # text = "Ours"
    # text_marker = create_text_marker_planner(text, x_start + scale_length/2 + 140, y_start + 5, 15, 5)
    # marker_array.markers.append(text_marker)

    # Line Marker
    line_marker1 = create_line_marker(x_start, y_start, x_start + float(scale_length), y_start, 0.5, 1, line_width)
    marker_array.markers.append(line_marker1)

    line_marker2 = create_line_marker(x_start, y_start - line_width/2, x_start, y_start + line_width/2 +1, 0.5, 2, line_width)
    marker_array.markers.append(line_marker2)

    line_marker3 = create_line_marker(x_start  + float(scale_length), y_start - line_width/2, x_start + float(scale_length), y_start  + line_width/2 +1, 0.5, 3, line_width)
    marker_array.markers.append(line_marker3)

    while not rospy.is_shutdown():
        # Update the timestamp of each marker
        for marker in marker_array.markers:
            marker.header.stamp = rospy.Time.now()

        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        scale_visualization_publisher()
    except rospy.ROSInterruptException:
        pass