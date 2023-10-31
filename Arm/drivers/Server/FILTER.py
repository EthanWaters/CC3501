import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from typing import Any
import queue
import struct
import time
import numpy as np
from scipy.signal import butter, filtfilt,lfilter

def butter_lowpass(cutoff, fs, order):
    nyquist = 0.5*fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a 
    
def butter_lowpass_filter(data, cutoff=1, fs=3, order=4):
    b, a = butter_lowpass(cutoff, fs, order)
    print(lfilter(b, a, data[0]))
    
    filtered_data = [lfilter(b, a, angle) for angle in data]
    return filtered_data
    
file_path  = "all_unfiltered.txt"

print("here")
# Read unfiltered data from the input file
with open(file_path, "r") as input_file:
    unfiltered_data = []
    for line in input_file:
        joint_angles = [float(angle) for angle in line.split(",")]
        unfiltered_data.append(joint_angles)


# Apply the Butterworth filter to each set of joint angles
# filtered_data = [butter_lowpass_filter(joint_angles) for joint_angles in unfiltered_data]

# Apply the Butterworth filter to all sets of joint angles
filtered_data = butter_lowpass_filter(unfiltered_data)


print(filtered_data)
output_file_path = "just_butter.txt"
# Write the filtered data to the output file
with open(output_file_path, "w") as output_file:
    for joint_angles in filtered_data:
        # Convert the filtered joint angles back to a string for saving
        print(joint_angles)
        formatted_data = ",".join([str(angle) for angle in joint_angles])
        output_file.write(f"{formatted_data}\n")


print("here")