#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import TransformStamped

rospy.init_node('detection_tf_broadcaster')
br = tf2_ros.TransformBroadcaster()

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

rospy.Subscriber('depthai_ros_msgs/SpatialDetectionArray', SpatialDetectionArray, callback)
rospy.spin()
