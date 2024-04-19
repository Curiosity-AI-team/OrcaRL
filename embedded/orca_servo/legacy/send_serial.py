import serial
import time

try:
    for i in range(0,4):
        ser = serial.Serial('/dev/ttyACM1', 115200)
        time.sleep(0.15) # Wait for the connection to establish
        count = 0
        responce_timeout = time.time()
        while True:
            start_time = time.time()
            data = "90,45,180,90,120,60,30,150,90,45,180,90,120,60,30,150,90,45,180,90\n"       
            ser.write(data.encode())
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
        print("Timeout!")
        time.sleep(0.1)
    print("Error")
    ser.close()

finally:
    ser.close() # Ensure the port is closed
