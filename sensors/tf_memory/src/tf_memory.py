#!/usr/bin/env python3


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import TransformStamped

def callback(data):
    for detection in data.detections:
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_frame"
        t.child_frame_id = "object_" + str(detection.id)

        t.transform.translation.x = detection.position.x
        t.transform.translation.y = detection.position.y
        t.transform.translation.z = detection.position.z

        # Assuming the object has no orientation information
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        br.sendTransform(t)

br = tf2_ros.TransformBroadcaster()

def main():
    rospy.init_node('tf2_listener_example')

    # Create a tfBuffer to store the transforms
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber('depthai_ros_msgs/SpatialDetectionArray', SpatialDetectionArray, callback)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # Lookup the transform from 'map' to 'aruco_marker_6_ext'
            trans = tfBuffer.lookup_transform('map', 'aruco_marker_6_ext', rospy.Time.now(), rospy.Duration(2.0))
            rospy.loginfo("Transform from 'map' to 'aruco_marker_6_ext': %s", trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not find transform: %s", e)
            rate.sleep()
            continue

        # Example usage of the transform
        # For demonstration, we'll just print the transform
        # In a real application, you might use this transform to control a robot or perform other operations

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
