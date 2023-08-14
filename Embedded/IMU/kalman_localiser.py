import os
import sys
import time
import smbus
import numpy as np
import zmq

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 

# # initializing publisher
# host = '192.168.1.101'
# port = 8358
# url = 'tcp://'+host+':'+str(port)
# context = zmq.Context()
# socket = context.socket(zmq.PUB)
# socket.bind(url)

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

imu.loadCalibDataFromFile("/home/pi/SensorFusion/PyFusion/CalibrationFiles/TEST_CAL_1.json")

sensorfusion = kalman.Kalman()

# imu.readSensor()
# imu.computeOrientation()
# sensorfusion.roll = imu.roll
# sensorfusion.pitch = imu.pitch
# sensorfusion.yaw = imu.yaw

count = 0
currTime = time.time()

while True:
    imu.readSensor()
    imu.computeOrientation()
    newTime = time.time()
    dt = newTime - currTime
    currTime = newTime

    sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
                                              imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
    
    yaw = sensorfusion.yaw
    if yaw < 0:
        yaw_positive = yaw + 360
    else:
        yaw_positive = yaw
        
    if yaw_positive <= 160:
        yaw_mapped = yaw_positive * 1.5
    elif (yaw_positive >= 190) & (yaw_positive < 340):
        yaw_mapped = yaw_positive * 0.95 #  (yaw_positive - 225 - 75) * 2.6 + 225        
    else:
        yaw_mapped = yaw_positive
    
    # print("Kalman: Roll:{0: 3.2f} ; Pitch:{1: 3.2f} ; Yaw:{2: 3.2f} ; {3: 3.1f}ms".format(sensorfusion.roll, sensorfusion.pitch, yaw, dt*1000))
    print(f"Roll : yaw-raw = {yaw:> 6.0f} ; yaw-pos = {yaw_positive:> 6.0f} ; yaw-map = {yaw_mapped:> 6.0f}")

    # md = dict(topic = 'orientation', kalman = str([sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw]))
    # socket.send_json(md)

    time.sleep(0.01)
