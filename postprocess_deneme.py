# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 19:05:33 2023

@author: ysfsl"""

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
counter = 0
hatalı_gps = []
buffer =  b''
def keyboard_interrupt_handler(signal_num, frame):
    
    print("KeyboardInterrupt received, running final code...")
    global counter, buffer, IMU_list, GPS_list
    global arr1_imu, arr2_gps, arr_acc, arr_quaternion, arr_RoR, arr_Mag, output_file, buffer, counter
    global IMU_list, GPS_list, arr_status, arr_lat_lon, arr_alt,arr_vel, arr_time, time_stamp_list, time_stamp_list_gps
    # print('here')
    buffer = bytearray(buffer)
    for i in range(len(buffer)-3):
        print(i)
        if (buffer[i] == 250) & (buffer[i+1] ==255) & (buffer[i+2] == 54):
            if buffer[i+3] == 83:
                counter += 1
                print(buffer[2], counter)
                print('len of buf',len(buffer))
                message = buffer[i:i+88]
                s = sum(message[1:])
                hex_code = hex(s)[2:]
                hex_lower_byte = hex_code[-2:]
                if hex_lower_byte == "00":
                    IMU_list.append(message)
                    # buffer -= buffer[start_index:start_index+88]
                else:
                    hatalı_list.append(message)
            elif buffer[i+3] == 128:
                # print('here')
                # while len(buffer) <  111:
                #     print('dolmadı')
                    # print(len(buffer))
                print(buffer[2])
                # gps+imu data
                message = buffer[i:i+133]
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
    GPS_list = GPS_list[10:]
    IMU_list = IMU_list[10:]
    arr1_imu = np.zeros((len(IMU_list),58)) 
    arr2_gps = np.zeros((len(GPS_list),94)) 
    # arr_acc = np.zeros((len(IMU_list),3))
    # arr_quaternion = np.zeros((len(IMU_list),4))
    # arr_RoR = np.zeros((len(IMU_list),3))
    # arr_Mag = np.zeros((len(IMU_list),3))
    # arr_status = np.zeros((len(IMU_list),4))
    time_stamp_list = []
    time_stamp_list_gps = []
    # arr_acc_gps = np.zeros((len(GPS_list),3))
    # arr_quaternion_gps = np.zeros((len(GPS_list),4))
    # arr_RoR_gps = np.zeros((len(GPS_list),3))
    # arr_Mag_gps = np.zeros((len(GPS_list),3))
    # arr_status_gps = np.zeros((len(GPS_list),4))
    # arr_lat_lon = np.zeros((len(GPS_list),2))
    # arr_alt = np.zeros((len(GPS_list),1))
    # arr_vel = np.zeros((len(GPS_list),3))
    
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
        # quaternion = msg[19:35]  # 
        # arr_quaternion[i][0:4] = struct.unpack('!4f', quaternion) 
        
        # imu_acceleration_bytes = msg[38:50]  # 
        # arr_acc[i][0:3] = struct.unpack('!3f', imu_acceleration_bytes) 
        
        arr1_imu[i][18:30] = msg[38:50] #Acceleration
        
        arr1_imu[i][30:42] = msg[53:65]
        # ror_temp = msg[53:65]  # 
        # arr_RoR[i][0:3] = struct.unpack('!3f', ror_temp) 

        arr1_imu[i][42:54] = msg[68:80]
        # mag_temp = msg[68:80]
        # arr_Mag[i][0:3] = struct.unpack('!3f', mag_temp) 
        
        arr1_imu[i][54:58] = msg[83:87]
        # status_temp =  msg[83:87]
        # arr_status[i][0:4] = status_temp
        
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
        # quaternion = msg[19:35]  # 
        # arr_quaternion_gps[i][0:4] = struct.unpack('!4f', quaternion) 
        
        # imu_acceleration_bytes = msg[38:50]  # 
        # arr_acc_gps[i][0:3] = struct.unpack('!3f', imu_acceleration_bytes) 
        
        arr2_gps[i][18:30] = msg[38:50] #Acceleration
        
        arr2_gps[i][30:42] = msg[53:65]
        # ror_temp = msg[53:65]  # 
        # arr_RoR_gps[i][0:3] = struct.unpack('!3f', ror_temp) 

        arr2_gps[i][42:54] = msg[68:80]
        # mag_temp = msg[68:80]
        # arr_Mag_gps[i][0:3] = struct.unpack('!3f', mag_temp) 
        
        arr2_gps[i][54:58] = msg[83:87]
        # status_temp =  msg[83:87]
        # arr_status_gps[i][0:1] = status_temp[-1]
        
        arr2_gps[i][58:70]= msg[90:102]
        # arr_lat_lon[i][0:2] = [fp1632_to_float64(msg[58:64]),fp1632_to_float64(msg[64:70])]
        
        arr2_gps[i][70:76] = msg[105:111]       
        # arr_alt[i][0:1] = fp1632_to_float64(msg[105:111])
        
        arr2_gps[i][76:] = msg[114:132]
        # arr_vel[i][0:3] = fp1632_to_float64(msg[114:132])       
        
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
        
    np.savetxt("GPS_results_test_post.csv", arr2_gps, delimiter=",")
    # np.savetxt("Lat_Lon_results_post.csv", arr_lat_lon, delimiter ="," )
    np.savetxt("IMU_results_test_post.csv", arr1_imu, delimiter =",")
    # Save frame data to a file
    t2 = time.time()
    ser.close()
    print('Time of process', t2-t1)
    print("Final code executed successfully.")
    exit(0)
    
signal.signal(signal.SIGINT, keyboard_interrupt_handler)

def MTI():
    global buffer
    # Check if there is data in the queue
    if not data_queue.empty():
        data = data_queue.get()
        
        buffer += data  
        # print(len(buffer))
        
while True: 
    MTI()