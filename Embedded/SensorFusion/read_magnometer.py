from threading import Thread
import time
import smbus
import sys
import numpy
import getpass

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 


def run_imu():
    global yaws
    global stop_threads
    global error_model

    # Sensor Stuff:
    address = 0x68
    bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(bus, address)
    imu.begin()
    imu.loadCalibDataFromFile("/home/pi/SensorFusion/PyFusion/CalibrationFiles/TEST_CAL_1.json")
    sensorfusion = kalman.Kalman()
    currTime = time.time()

    print("IMU Active")

    while not stop_threads:
        imu.readSensor()
        imu.computeOrientation()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
                                                imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
        
        yaw = sensorfusion.yaw

        # Define positive yaw (0 -> 360)
        yaw_pos = (yaw + 360) % 360 if yaw < 0 else yaw

        # Map yaw using polynomial funciton:
        yaw_mapped = yaw_pos * error_model(yaw_pos)

        yaws = {'raw': yaw, 'pos': yaw_pos, 'map': yaw_mapped}

        time.sleep(0.01)


def map_mag():
    global yaws
    global error_model

    while 'yaws' not in globals():
        time.sleep(0.1)

    while True:
            print(f"{yaws['pos']:<3.1f}  x  {error_model(yaws['pos']):<4.2f}  =  {yaws['map']:<3.1f}")
            time.sleep(0.015)
            

### START ###
def main():
    global stop_threads
    global error_model

    error_model = numpy.poly1d([-8.84018876e-10,  7.67953538e-07, -2.16196333e-04,  1.92186193e-02,  1.08444277e+00])

    Thread(target=run_imu).start()
    print("Starting IMU...")

    try:
        stop_threads = False
        map_mag()
    finally:
        stop_threads = True


if __name__ == "__main__":
    main()
