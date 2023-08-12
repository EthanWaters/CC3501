import os
import sys
import time
import smbus
import zmq

from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick

# initializing publisher
host = '192.168.1.101'
port = 8358
url = 'tcp://'+host+':'+str(port)
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(url)

sensorfusion = madgwick.Madgwick(0.5)

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# imu.caliberateGyro()
# imu.caliberateAccelerometer()
# or load your own caliberation file
# imu.loadCalibDataFromFile("/home/pi/calib_real4.json")

currTime = time.time()
print_count = 0

while True:
    imu.readSensor()
    for i in range(10):
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
                                    imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

    if print_count == 2:
        print("Magwick: roll: {0: .2f} ; pitch: {1: 3.2f} ; yaw: {2: 3.2f}".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))
        
        md = dict(topic = 'orientation', madgwick = str([sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw]))
        socket.send_json(md)
        
        print_count = 0

    print_count = print_count + 1
    time.sleep(0.01)
