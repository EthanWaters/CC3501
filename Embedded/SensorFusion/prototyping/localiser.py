import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
# imu.caliberateGyro()
# imu.caliberateAccelerometer()
# or load your own caliberation file
# imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

while True:
	imu.readSensor()
	imu.computeOrientation()

	print ("roll: {0: .2f} ; pitch : {1: .2f} ; yaw : {2: .2f}".format(imu.roll, imu.pitch, imu.yaw))
	time.sleep(0.1)
