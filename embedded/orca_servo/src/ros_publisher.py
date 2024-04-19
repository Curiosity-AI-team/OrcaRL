#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

def serial_to_ros():
   pub = rospy.Publisher('servo_command', Int32MultiArray, queue_size=10)
   rospy.init_node('ros_publisher', anonymous=False)

   while not rospy.is_shutdown():
       msg = Int32MultiArray()
       msg.data = [99,46,180,90,120,60,30,150,90,45,180,90,120,60,30,150,90,45,180,90]
       pub.publish(msg)
       rospy.sleep(0.1)

if __name__ == '__main__':
   try:
       serial_to_ros()
   except rospy.ROSInterruptException:
       pass