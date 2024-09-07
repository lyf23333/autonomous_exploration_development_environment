#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def create_circle_marker(x, y, z, radius, marker_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "circle"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = radius * 2  # Diameter
    marker.scale.y = radius * 2  # Diameter
    marker.scale.z = 1.0  # Small thickness to represent a circle
    marker.color.a = 1.0  # Transparency
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker

def create_text_marker(text, x, y, z, marker_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "text_marker"
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
    marker.scale.x = 10
    marker.scale.y = 10
    marker.scale.z = 10  # Height of the text
    marker.color.a = 1.0  # Transparency
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.text = text

    return marker

def circle_publisher():
    rospy.init_node('circle_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_starts_marker', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    marker_array = MarkerArray()

    pub_name = rospy.Publisher('visualization_name_marker', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    marker_array_names = MarkerArray()
    
    radius = 1.5
    positions_list = [
        [0.0, 0.0, 0.0],
        [123.5, 50.3, 0],
        [72.5, -26.6, 0.0],
        [4.25, 41.7, 0.0],
        [6.2, -17.9, 0.0],
        [81.6, 52.3, 0.0],
        [39.6, -24.2, 0.0]
    ]
    positions_names_list = [
        [0.0 - 13.0, 0.0, 0.0],
        [123.5,      50.3 + 10, 0],
        [72.5,       -26.6 - 10, 0.0],
        [4.25,       41.7 + 10, 0.0],
        [6.2,        -17.9 - 10, 0.0],
        [81.6,       52.3 + 10, 0.0],
        [39.6,       -24.2 - 10, 0.0]
    ]
    text_lists = ["start", "1", "2", "3", "4", "5", "6"]
    for i in range(len(positions_list)):
        x = positions_list[i][0]
        y = positions_list[i][1]
        z = positions_list[i][2]
        marker = create_circle_marker(x, y, z, radius, i)
        marker_array.markers.append(marker)

        x = positions_names_list[i][0]
        y = positions_names_list[i][1]
        z = positions_names_list[i][2]
        text = text_lists[i]
        marker = create_text_marker(text, x, y, z, i)
        marker_array_names.markers.append(marker)

    while not rospy.is_shutdown():
        pub.publish(marker_array)

        # Update the timestamp of each marker
        for marker in marker_array_names.markers:
            marker.header.stamp = rospy.Time.now()
        pub_name.publish(marker_array_names)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle_publisher()
    except rospy.ROSInterruptException:
        pass
