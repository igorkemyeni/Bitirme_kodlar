import serial
import numpy as np
import time
import threading
import concurrent.futures
import queue
import datetime
import struct
import signal
import cv2
from collections import namedtuple
import pickle
import random
import sys
import csv

from VisualOdometryRealTime import VO, get_calib

Data_list = []
Imu_data = bytearray()
IMU_list = []
GPS_list = []
hatalı_list = []
time_list = []
t1 = time.time()  

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
  


# Create a Queue object to hold the incoming data
# data_queue = queue.Queue()


exit_event = threading.Event()


# Define the thread function to read data from the serial port
def read_serial_data(ser):
    global IMU_list
    while not exit_event.is_set():
        s = 0
        data = ser.read(1)
        # print(data)
        if data == b"\xfa":
             data = ser.read(1)
             if data == b"\xff":
                 s += data[0]
                 data = ser.read(1)
                 if data == b"\x36":
                     s += data[0]
                     data = ser.read(1)
                     if data == b"\x53":
                         s+= data[0]
                         data = ser.read(84)
                         s+= sum(data[0:])
                         hex_code = hex(s)[-2:]
                         hex_lower_byte = hex_code[-2:]
                         if hex_lower_byte == "00":
                             
                              print("imu",data[3],data[4])
                              imu_parsed = imu_data_parse(data)
                              Data_list.append(imu_parsed)
                              IMU_list.append(imu_parsed)
                         else:
                             print('Imu data is corrupted, skip packet: ', data[3],data[4])
                         
                     elif data == b"\x80":
                         s+= data[0]
                         data = ser.read(129)
                         s+= sum(data[0:])
                         hex_code = hex(s)[-2:]
                         hex_lower_byte = hex_code[-2:]
                         if hex_lower_byte == "00":
                             
                             
                             print("gps",data[3],data[4])
                         
                             GPS_parsed = GPS_data_parse(data)
                             Data_list.append(GPS_parsed)
                             GPS_list.append(GPS_parsed)
                         else:
                             print('GPS data is corrupted, skip packet: ', data[3],data[4])
                     
                     else:
                        print('4. ifte hata')
                 else:
                     print('3. ifte hata')        # data_queue.put(data)
             else:
                 print('2. ifte hata')   
        else:
            print('ilk ifte hata')
    
# def signal_handler(signum, frame):
#     print("KeyboardInterrupt received, stopping the serial thread...")
#     exit_event.set()
#     serial_thread.join()  # Wait for the child thread to finish before the main thread exits
#     print("Serial thread stopped, exiting the program.")
#     # sys.exit(0)  # End the program

def time_stamp(data): 
    global time_list
    max_value = 0xFFFFFFFFF
    # print(data)
    sample_time = data[0]
    # print('sampt',sample_time)
    
    if sample_time >= max_value:
        # wraparound occurred, subtract max_value and add one day
        elapsed_time = (sample_time - max_value) / 10000
        timestamp = datetime.datetime.now() - datetime.timedelta(days=1) + datetime.timedelta(seconds=elapsed_time)
    else:   
        # no wraparound, convert directly to datetime.datetime object
        timestamp = datetime.datetime.now() + datetime.timedelta(seconds=sample_time / 10000)

    # format the timestamp as a string with milliseconds
    timestamp_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
    print(timestamp_str)
    time_list.append(timestamp_str)
    return timestamp_str

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

def imu_data_parse(data):
        # TİMESTAMPLAR YANLIŞ GELİYOR DÜZELT
        
        time_temp = data[8:12]
        time_temp = struct.unpack('!I', time_temp)
        ts = time_stamp(time_temp)
        quaternion = data[15:31]  # 
        imu_quaternion = struct.unpack('!4f', quaternion) 
        
        imu_acceleration_bytes = data[34:46]  # 
        imu_acc = struct.unpack('!3f', imu_acceleration_bytes) 
        
        ror_temp = data[49:61]  # 
        imu_rot = struct.unpack('!3f', ror_temp) 
        
        mag_temp = data[64:76]
        imu_Mag = struct.unpack('!3f', mag_temp) 
        
        # imu_status =  data[79:83]
        
        total = imu_quaternion +imu_acc+imu_rot+imu_Mag + (ts,)
        return total

    
def GPS_data_parse(data):
    # print('parse girdi', len(data))
    time_temp = data[8:12]
    time_temp = struct.unpack('!I', time_temp)
    ts = time_stamp(time_temp)
    quaternion = data[15:31]  # 
    imu_quaternion = struct.unpack('!4f', quaternion)
    # print(type(imu_quaternion))
    
    imu_acceleration_bytes = data[34:46]  # 
    imu_acc = struct.unpack('!3f', imu_acceleration_bytes) 
    # print(type(imu_acc))
    
    ror_temp = data[49:61]  # 
    imu_rot = struct.unpack('!3f', ror_temp) 
    # print(type(imu_rot))
    
    mag_temp = data[64:76]
    imu_Mag = struct.unpack('!3f', mag_temp)
    # print(type(imu_Mag))
    
    
    # imu_status =  data[80:84]
    
    GPS_lat_lon = (fp1632_to_float64(data[86:92]),fp1632_to_float64(data[92:98]))
    # print(type(GPS_lat_lon))
    GPS_alt = (fp1632_to_float64(data[101:107]),)
    # print(type(GPS_alt))
    
    GPS_vel = (fp1632_to_float64(data[110:128]),)  
    # print(type(GPS_vel))
    
    
    total = imu_quaternion +imu_acc+imu_rot+imu_Mag+ (ts,) + GPS_lat_lon + GPS_alt + GPS_vel 
    # print(type(total))
    return total

stop_sig = 0
frame_data_list = []
timelist_zed = list()

rt_frame_data_list = []
rt_timelist_zed = list()

def handle_zed_camera(device_id):
    global frame_data_list, timelist_zed, rt_frame_data_list, rt_timelist_zed
    global stop_sig
    

    cap = cv2.VideoCapture(device_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    while cap.isOpened() and not exit_event.is_set():
        ret, frame = cap.read()
        if ret:
            timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')
            # frame_data = FrameData(timestamp, frame)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rt_frame_data_list.append(gray_frame)
            frame_data_list.append(gray_frame)
            # frame_data_list.append(frame)
            rt_timelist_zed.append(timestamp)
            timelist_zed.append(timestamp)

            cv2.imshow('ZED Camera', frame)

            if cv2.waitKey(1) & 0xFF == stop_sig:
                break
    cap.release()
    cv2.destroyAllWindows()
    
def run_main(zed_cam_id):
    global Data_list, frame_data_list, time_list, timelist_zed
    # Start threads for both devices
    
    P0, P1 = get_calib()
    
    zed_thread = threading.Thread(target=handle_zed_camera, args=(zed_cam_id,))
    serial_thread = threading.Thread(target=read_serial_data, args=(ser,))
    
    
    
    
    executor = concurrent.futures.ThreadPoolExecutor()
    
    vo_thread = threading.Thread(target=vo.run, args=(rt_frame_data_list[1],))
    
    
    zed_thread.start()
    serial_thread.start()
    
    vo = VO(P0, P1, rt_frame_data_list[0])
    
    while not exit_event.is_set():
        a= input('Enter qa(to not save) or qw (to save) to exit:')
        if a == 'qw':
            ser.close()
            exit_event.set()
            Data_list = Data_list[10:]
            time_list = time_list[10:]
            t2 = time.time()
            print('Time of process', t2-t1)
            print("Final code executed successfully.")
            return Data_list, frame_data_list,time_list, timelist_zed
        
        elif a == 'qa':
            ser.close()
            exit_event.set()
            t2 = time.time()
            print('Time of process', t2-t1)
            print("Final code executed successfully.")
        
    
        
    
# stop_thread.start()
# serial_thread.join()
# serial_thread.run()

    # np.savetxt("GPS_results_test.csv", arr2_gps, delimiter=",")
    # np.savetxt("Lat_Lon_results.csv", arr_lat_lon, delimiter ="," )
    # np.savetxt("IMU_results_test.csv", arr1_imu, delimiter =",")
# def keyboard_interrupt_handler(signal_num, frame):
#     global serial_threadv
#     print("KeyboardInterrupt received, running final code...")
#     # global arr1_imu, arr2_gps, arr_acc, arr_quaternion, arr_RoR, arr_Mag, output_file
#     # global IMU_list, GPS_list, arr_status, arr_lat_lon, arr_alt,arr_vel, arr_time, time_stamp_list, time_stamp_list_gps
    
#     """ Arr1 contains all the necessary data for the IMU data
#     * First 2 Columns contains Packet Counter
#     * Next 16 Columns (2:18) contains Quaternion
#     * Next 12 Bytes (18:30) contains Acceleration
#     * Next 12 Bytes (30:42) contains RoR
#     * Next 12 bytes (42:54) contains magnetic field
#     * Next 4 bytes (54:58) contains status word 
#     For the GPS Data
#     * Neft 12 Bytes (58:70) contains Lat Long (fp1632)
#     * Next 6 bytes (70:76) contains altitude (fp1632)
#     """

#     # np.savetxt("GPS_results_test.csv", arr2_gps, delimiter=",")
#     # np.savetxt("Lat_Lon_results.csv", arr_lat_lon, delimiter ="," )
#     # np.savetxt("IMU_results_test.csv", arr1_imu, delimiter =",")
#     # Save frame data to a file
#     # with open(output_file, 'wb') as f:
#         # pickle.dump(frame_data_list, f)
#     t2 = time.time()
#     serial_thread.join()
#     ser.close()
#     print('Time of process', t2-t1)
#     print("Final code executed successfully.")
#     exit(0)

# while True:
#     signal.signal(signal.SIGINT, keyboard_interrupt_handler)
