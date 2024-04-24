#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from depthai_ros_msgs.msg import SpatialDetectionArray
import numpy as np
import json
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def check_point_in_pyramid(pyramid, point):
    """
    Determines if a point is inside a square-based pyramid.
    
    Parameters:
    pyramid: Dictionary containing the vertices of the pyramid.
    point: The point to check (PointStamped).
    
    Returns:
    True if the point is inside the pyramid, False otherwise.
    """
    A, B, C, D, E = [np.array(pyramid[key]) for key in ['A', 'B', 'C', 'D', 'E']]
    Point = np.array([point.point.x, point.point.y, point.point.z])
    matrix = np.array([A, B, C, D, E]).T

    try:
        barycentric_coords = np.linalg.solve(matrix, Point)
    except np.linalg.LinAlgError:
        return False

    if np.all(barycentric_coords >= 0) and np.isclose(np.sum(barycentric_coords), 1):
        return True
    else:
        return False

def callback(msg, args):
    br, tfBuffer, data, pyramid, depth_range = args
    for detection in msg.detections:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_frame"
        t.child_frame_id = "object_" + str(detection.id)
        t.transform.translation.x = detection.position.x
        t.transform.translation.y = detection.position.y
        t.transform.translation.z = detection.position.z
        t.transform.rotation.w = 1
        br.sendTransform(t)

        try:
            transform = tfBuffer.lookup_transform('map', 'camera_frame', rospy.Time.now(), rospy.Duration(1.0))
            transformed_point = tf2_geometry_msgs.do_transform_point(detection.position, transform)
            if check_point_in_pyramid(pyramid, transformed_point) and (depth_range[0] <= transformed_point.point.z <= depth_range[1]):
                data[f"object_{detection.id}"] = {'x': transformed_point.point.x, 'y': transformed_point.point.y, 'z': transformed_point.point.z}
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not transform detection: %s", e)

def save_point_data(data):
    file_path = '/home/vboxuser/test_ws/src/OrcaRL/sensors/tf_memory/database/data.json'
    with open(file_path, 'w') as f:
        json.dump(data, f)

def load_point_data():
    file_path = '/home/vboxuser/test_ws/src/OrcaRL/sensors/tf_memory/database/data.json'
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            return json.load(f)
    return {}

def main():
    rospy.init_node('tf2_listener_example')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    data = load_point_data()
    pyramid = {'A': [0, 0, 0], 'B': [1, 0, 0], 'C': [0, 1, 0], 'D': [1, 1, 0], 'E': [0.5, 0.5, 1]}
    depth_range = [0, 10]
    rospy.Subscriber('depthai_ros_msgs/SpatialDetectionArray', SpatialDetectionArray, callback, (br, tfBuffer, data, pyramid, depth_range))
    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(10.0)
    markerArray = MarkerArray()
    count = 0
    MARKERS_MAX = 100

    while not rospy.is_shutdown():
        save_point_data(data)
        markerArray.markers = []

        for key, value in data.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position = Point(value['x'], value['y'], value['z'])
            marker.id = int(key.split('_')[1])

            markerArray.markers.append(marker)

        # Ensure we don't exceed the maximum number of markers
        if len(markerArray.markers) > MARKERS_MAX:
            markerArray.markers = markerArray.markers[-MARKERS_MAX:]

        # Renumber the marker IDs
        for i, m in enumerate(markerArray.markers):
            m.id = i

        pub.publish(markerArray)
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
