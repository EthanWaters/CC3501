import smbus
import time

from imusensor.MPU9250 import MPU9250


address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

print ("Magnomter Calibration Starting, spin me right round baby like a recond baby right round round round.")
time.sleep(2)

imu.caliberateMagPrecise()
print ("Mag calibration Finished")
print (imu.MagBias)
print (imu.Magtransform)
print (imu.Mags)

imu.saveCalibDataToFile("/home/pi/SensorFusion/PyFusion/CalibrationFiles/MAG_CALIBRATION.json")