"""
Created on Sun Mar 12 21:49:56 2023

@author: ysfsl
"""
import serial
import numpy as np
import time
import threading
import queue
import datetime
import struct
import signal
import cv2
from collections import namedtuple
import pickle

factorFP1632 = 1 << 32
settings = {
    'port': 'COM3',
    'baudrate': 921600,
    'timeout': 1,
    'bytesize': serial.EIGHTBITS,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_ONE,
    'xonxoff': False,
    'rtscts': False,
    'dsrdtr': False,
    'writeTimeout': None,
    'interCharTimeout': None,
    'exclusive': None
}

# Configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(**settings)
# Send command to MTI7 IMU

# 1. Send command to MTI7 IMU to get the data
# thestring1 = FA FF 30 00 D1
str1 = b"\xfa\xff\x30\x00\xd1"
ser.write(str1)

config_str = bytes.fromhex("FA FF C0 28 10 20 FF FF 10 60 FF FF 20 10 00 64 40 20 01 90 80 20 01 90 C0 20 00 64 E0 20 FF FF 50 42 00 64 50 22 00 64 D0 12 00 64 73")
ser.write(config_str)
# Send command to MTI7 IMU to get the data
str4 = b"\xfa\xff\x10\x00\xf1"
ser.write(str4)
  
GpsImu_data = bytearray()
Imu_data = bytearray()
IMU_list = []
GPS_list = []
hatalı_list = []
t1 = time.time()  

# Create a Queue object to hold the incoming data
data_queue = queue.Queue()



# Define the thread function to read data from the serial port
def read_serial_data(ser, data_queue):
    while True:
        data = ser.read(132)
        data_queue.put(data)

# Create a thread to run the read_serial_data function and set it as a daemon thread
serial_thread = threading.Thread(target=read_serial_data, args=(ser, data_queue))
serial_thread.setDaemon(True)
serial_thread.start()

def fp1632_to_float64(fp):
    d = bytearray([0, 0, fp[4], fp[5], fp[0], fp[1], fp[2], fp[3]])
    if d[2] & 0x80:
        # sign-extend to 64 bits
        d[0] = 0xff
        d[1] = 0xff
    u = struct.unpack('>Q', d)[0]
    i = struct.unpack('>q', struct.pack('>Q', u))[0]  # reinterpret as signed
    factor = 1 << 32
    f = float(i) / factor
    return f

def keyboard_interrupt_handler(signal_num, frame):
    print("KeyboardInterrupt received, running final code...")
    global arr1_imu, arr2_gps, arr_acc, arr_quaternion, arr_RoR, arr_Mag, output_file
    global IMU_list, GPS_list, arr_status, arr_lat_lon, arr_alt,arr_vel, arr_time, time_stamp_list, time_stamp_list_gps
    arr1_imu = np.zeros((len(IMU_list),58)) 
    arr2_gps = np.zeros((len(GPS_list),94)) 
    arr_acc = np.zeros((len(IMU_list),3))
    arr_quaternion = np.zeros((len(IMU_list),4))
    arr_RoR = np.zeros((len(IMU_list),3))
    arr_Mag = np.zeros((len(IMU_list),3))
    arr_status = np.zeros((len(IMU_list),4))
    time_stamp_list = []
    time_stamp_list_gps = []
    arr_acc_gps = np.zeros((len(GPS_list),3))
    arr_quaternion_gps = np.zeros((len(GPS_list),4))
    arr_RoR_gps = np.zeros((len(GPS_list),3))
    arr_Mag_gps = np.zeros((len(GPS_list),3))
    arr_status_gps = np.zeros((len(GPS_list),4))
    arr_lat_lon = np.zeros((len(GPS_list),2))
    arr_alt = np.zeros((len(GPS_list),1))
    arr_vel = np.zeros((len(GPS_list),3))
    
    arr_time = np.zeros((len(IMU_list),1))
    """ Arr1 contains all the necessary data for the IMU data
    * First 2 Columns contains Packet Counter
    * Next 16 Columns (2:18) contains Quaternion
    * Next 12 Bytes (18:30) contains Acceleration
    * Next 12 Bytes (30:42) contains RoR
    * Next 12 bytes (42:54) contains magnetic field
    * Next 4 bytes (54:58) contains status word 
    For the GPS Data
    * Neft 12 Bytes (58:70) contains Lat Long (fp1632)
    * Next 6 bytes (70:76) contains altitude (fp1632)
    """
    for i,msg in enumerate(IMU_list):
        time_temp = msg[12:16]
        arr_time[i] = struct.unpack('!I', time_temp)
        
        arr1_imu[i][0:2] = msg[7:9]    
        arr1_imu[i][2:18] = msg[19:35]  
        quaternion = msg[19:35]  # 
        arr_quaternion[i][0:4] = struct.unpack('!4f', quaternion) 
        
        imu_acceleration_bytes = msg[38:50]  # 
        arr_acc[i][0:3] = struct.unpack('!3f', imu_acceleration_bytes) 
        
        arr1_imu[i][18:30] = msg[38:50] #Acceleration
        
        arr1_imu[i][30:42] = msg[53:65]
        ror_temp = msg[53:65]  # 
        arr_RoR[i][0:3] = struct.unpack('!3f', ror_temp) 

        arr1_imu[i][42:54] = msg[68:80]
        mag_temp = msg[68:80]
        arr_Mag[i][0:3] = struct.unpack('!3f', mag_temp) 
        
        arr1_imu[i][54:58] = msg[83:87]
        status_temp =  msg[83:87]
        arr_status[i][0:4] = status_temp
        
        max_value = 0xFFFFFFFFF
        sample_time = arr_time[i].item()
        
        if sample_time >= max_value:
            # wraparound occurred, subtract max_value and add one day
            elapsed_time = (sample_time - max_value) / 10000
            timestamp = datetime.datetime.now() - datetime.timedelta(days=1) + datetime.timedelta(seconds=elapsed_time)
        else:   
            # no wraparound, convert directly to datetime.datetime object
            timestamp = datetime.datetime.now() + datetime.timedelta(seconds=sample_time / 10000)

        # format the timestamp as a string with milliseconds
        timestamp_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
        time_stamp_list.append(timestamp_str)
            
            
            
            
    for i,msg in enumerate(GPS_list):
        time_temp = msg[12:16]
        arr_time[i] = struct.unpack('!I', time_temp)
        arr2_gps[i][0:2] = msg[7:9]    
        arr2_gps[i][2:18] = msg[19:35]  
        quaternion = msg[19:35]  # 
        arr_quaternion_gps[i][0:4] = struct.unpack('!4f', quaternion) 
        
        imu_acceleration_bytes = msg[38:50]  # 
        arr_acc_gps[i][0:3] = struct.unpack('!3f', imu_acceleration_bytes) 
        
        arr2_gps[i][18:30] = msg[38:50] #Acceleration
        
        arr2_gps[i][30:42] = msg[53:65]
        ror_temp = msg[53:65]  # 
        arr_RoR_gps[i][0:3] = struct.unpack('!3f', ror_temp) 

        arr2_gps[i][42:54] = msg[68:80]
        mag_temp = msg[68:80]
        arr_Mag_gps[i][0:3] = struct.unpack('!3f', mag_temp) 
        
        arr2_gps[i][54:58] = msg[83:87]
        status_temp =  msg[83:87]
        arr_status_gps[i][0:1] = status_temp[-1]
        
        arr2_gps[i][58:70]= msg[90:102]
        arr_lat_lon[i][0:2] = [fp1632_to_float64(msg[58:64]),fp1632_to_float64(msg[64:70])]
        
        arr2_gps[i][70:76] = msg[105:111]       
        arr_alt[i][0:1] = fp1632_to_float64(msg[105:111])
        
        arr2_gps[i][76:] = msg[114:132]
        arr_vel[i][0:3] = fp1632_to_float64(msg[114:132])       
        
        max_value = 0xFFFFFFFFF
        sample_time = arr_time[i].item()
        
        if sample_time >= max_value:
            # wraparound occurred, subtract max_value and add one day
            elapsed_time = (sample_time - max_value) / 10000
            timestamp = datetime.datetime.now() - datetime.timedelta(days=1) + datetime.timedelta(seconds=elapsed_time)
        else:   
            # no wraparound, convert directly to datetime.datetime object
            timestamp = datetime.datetime.now() + datetime.timedelta(seconds=sample_time / 10000)

        # format the timestamp as a string with milliseconds
        timestamp_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
        time_stamp_list_gps.append(timestamp_str)
        
    np.savetxt("GPS_results_test.csv", arr2_gps, delimiter=",")
    np.savetxt("Lat_Lon_results.csv", arr_lat_lon, delimiter ="," )
    np.savetxt("IMU_results_test.csv", arr1_imu, delimiter =",")
    # Save frame data to a file
    with open(output_file, 'wb') as f:
        pickle.dump(frame_data_list, f)
    t2 = time.time()
    ser.close()
    print('Time of process', t2-t1)
    print("Final code executed successfully.")
    exit(0)
    # print(asdas2d)
whole_packets = []
# Set the keyboard interrupt handler
signal.signal(signal.SIGINT, keyboard_interrupt_handler)
counter = 0
buffer = bytearray()
hatalı_gps = []
def MTI():
    global IMU_list, GPS_list, counter, whole_packets, hatalı_gps,buffer
    # Check if there is data in the queue
    if not data_queue.empty():
        data = data_queue.get()
        
        buffer = bytearray(data)  # reset buffer with new data
        print(len(buffer))
        # Search for the start and end markers in the buffer
        start_index = buffer.find(b"\xfa")
        next_index = buffer.find(b"\xff")
        msg_id = buffer.find(b"\x36")
        # print(len(buffer))
        if (start_index > 0) & (start_index + 1 == next_index) & (start_index + 2 == msg_id):
            print('girdik')
            buffer = buffer[start_index:]
            whole_packets.append(buffer)
            if len(buffer)>3:
                if buffer[3] == 83 :
                    # while len(buffer) < 88:
                    #     print('ımu buf dolmadı', len(buffer))
                    if len(buffer) >= 88:
                        counter += 1
                        print(buffer[2], counter)
                        print('len of buf',len(buffer))
                        message = buffer[:88]
                        s = sum(message[1:])
                        hex_code = hex(s)[2:]
                        hex_lower_byte = hex_code[-2:]
                        if hex_lower_byte == "00":
                            IMU_list.append(message)
                            # buffer -= buffer[start_index:start_index+88]
                        else:
                            hatalı_list.append(message)
                        # imu data
                        buffer = bytearray()   # reset buffer
                            
                        
                       
                elif buffer[3] == 128:
                    print('here')
                    # while len(buffer) <  111:
                    #     print('dolmadı')
                        # print(len(buffer))
                    if len(buffer) >= 133: 
                        print(buffer[2])
                        # gps+imu data
                        message = buffer[:133]
                        # GpsImu_data = message[4:111]
                        s = sum(message[1:])
                        hex_code = hex(s)[2:]
                        hex_lower_byte = hex_code[-2:]
                        if hex_lower_byte == "00":
                            GPS_list.append(message)
                        else:
                            hatalı_gps.append(message)
                        buffer = bytearray()  # reset buffer

                    # else:
                    #     print('ekledik')
                    #     buffer.extend(data)

                else:
                    print("hatalı", buffer[3])
                    hatalı_list.append(buffer)
                    # buffer = bytearray()
                        
                        
                        
def handle_zed_camera(device_id, output_file):
    global frame_data_list
    

    cap = cv2.VideoCapture(device_id)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')
            frame_data = FrameData(timestamp, frame)
            frame_data_list.append(frame_data)

            cv2.imshow('ZED Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()
    

frame_data_list = []
FrameData = namedtuple('FrameData', ['timestamp', 'frame'])

# Configure your MTI-7 port and ZED camera device ID
# mti7_port = "COM3"  # Change this to your MTI-7 device port
zed_camera_device_id = 1  # Change this if your ZED camera has a different device ID
output_file = "frame_data.pkl"

# Start threads for both devices
# mti7_thread = threading.Thread(target=handle_mti7_data, args=(mti7_port,))
zed_thread = threading.Thread(target=handle_zed_camera, args=(zed_camera_device_id, output_file))
# mti7_thread.start()
zed_thread.start()

# Main program loop
counter = 0
while True: 
    MTI()
    







# Wait for both threads to finish
# mti7_thread.join()
# zed_thread.join()

    
            

              
"""
DECIMAL TO HEX CONV FOR Checksum check
for i in IMU_list:
    s = sum(i[1:])
    hex_code = hex(s)[2:]
    hex_lower_byte = hex_code[-2:]
    print("Lower byte of hex number", hex_code, "is", hex_lower_byte)
"""


