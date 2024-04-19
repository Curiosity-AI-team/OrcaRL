#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import serial
import time

data_list = None
def callback(data):
    global data_list
    data_list = ','.join(str(element) for element in data.data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    rospy.init_node('array_subscriber', anonymous=False)
    rospy.Subscriber("servo_command", Int32MultiArray, callback)
    rate = rospy.Rate(100)
    
    try:
        for i in range(0,40):
            if rospy.is_shutdown():
                break
            ser = serial.Serial('/dev/ttyACM1', 115200)
            time.sleep(0.15) # Wait for the connection to establish
            count = 0
            responce_timeout = time.time()
            while not rospy.is_shutdown():
                if data_list is not None:
                    start_time = time.time()
                    # data_list = "90,45,180,90,120,60,30,150,90,45,180,90,120,60,30,150,90,45,180,90\n"
                    ser.write(data_list.encode())
                    response = ser.readline() # this code run little faster than ser.readline()
                    if ((start_time - responce_timeout) > 0.01):
                        break
                    if ';' in response.decode():
                        feedback, analog_data = str(response.decode()).split(';', 1) # Split the data into feedback and analog parts
                        feedback = feedback.strip()
                        analog_data = analog_data.strip()
                        print(f"{feedback}::{analog_data}")
                        responce_timeout =  time.time()
                    
                    # # Print the time difference
                    time_diff = time.time() - start_time
                    print(f"Time difference: {time_diff} seconds")
                rate.sleep()
            print("Timeout!")
            time.sleep(0.01)
        print("Error")
        ser.close()

    finally:
        ser.close() # Ensure the port is closed