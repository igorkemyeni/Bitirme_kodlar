# -*- coding: utf-8 -*-

from function_real_time_deneme_v1_2 import run_main
import matplotlib.pyplot as plt
import time
import datetime
import csv

entry, Data_list, frame_data_list, time_list, timelist_zed, pos_ = run_main(0)

"""
runfile('C:/Users/ysfsl/Documents/GitHub/Bitirme_kodlar/rt_run.py', wdir='C:/Users/ysfsl/Documents/GitHub/Bitirme_kodlar')
Reloaded modules: VisualOdometryRealTime

OMP: Error #15: Initializing libiomp5md.dll, but found libiomp5md.dll already initialized.
OMP: Hint This means that multiple copies of the OpenMP runtime have been linked into the program. That is dangerous, since it can degrade performance or cause incorrect results. The best thing to do is to ensure that only a single OpenMP runtime is linked into the process, e.g. by avoiding static linking of the OpenMP runtime in any library. As an unsafe, unsupported, undocumented workaround you can set the environment variable KMP_DUPLICATE_LIB_OK=TRUE to allow the program to continue to execute, but that may cause crashes or silently produce incorrect results. For more information, please see http://www.intel.com/software/products/support/.
"""
if entry != 'qa':

    day = datetime.datetime.now().strftime("%D").replace("/", "_")
    time_ = timelist_zed[0].split('.')[0].replace(":", "_")
    
    ts = day + "_" + time_
    
    if entry == 'qvo':
        trajectory_nolidar_bm = pos_[0]
        fig = plt.figure(figsize=(6,6))
        ax = fig.add_subplot(111)#, projection='3d')
        
        ax.plot(
            trajectory_nolidar_bm[:, :, 3][:, 0], 
           #     trajectory_nolidar_bm[:, :, 3][:, 1], 
                trajectory_nolidar_bm[:, :, 3][:, 2],
                label='estimated', color='orange')
        
        ax.set_xlabel('x(m)')
        ax.set_ylabel('y(m)')
        
        plt.savefig(f"outputlar\\vo_{ts}.png")
    
    filename = f"outputlar\\imu_gps_{ts}_.csv"
    with open(filename, "w", newline="") as csv_file:
        # Create a CSV writer object
        writer = csv.writer(csv_file)
        
        # Write the header row
        writer.writerow(["q1","q2","q3","q4","ax","ay","az","RotX","RotY","RotZ","mgx","mgy","mgz","ts","lat","lon","alt","vx", "vy", "vz"])
        
        # Write each tuple as a row in the CSV file
        for row in Data_list:
            writer.writerow(row)