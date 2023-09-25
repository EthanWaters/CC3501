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

    error_factors = []
    angles_true = []
    angles_true.extend(range(0, 360, 30))
    
    output_file = open('mag_map.csv', 'w+', newline='')

    while 'yaws' not in globals():
        time.sleep(0.1)

    for angle_true in angles_true:
        
        # Strings:
        input_string = "Compute for "
        output_spacer = len(input_string) * ' '

        # Display prompt:
        print(f"{input_string}{angle_true:<3}: ", end='', flush=True)
        getpass.getpass(prompt='')  # Wait for the user to press Enter.

        # Get data:
        angle_mag = yaws['pos']
        error_factor = angle_true / angle_mag
        error_factors.append(error_factor)

        # Print result:
        print(f"{output_spacer}{angle_mag:<3.1f} \n", end='')

        # Write result:
        result = f"{angle_true}\t {int(angle_mag)}\t {error_factor:.2f}\n"
        output_file.write(result)

    # Calculate a polynomial regression:
    x = angles_true
    y = error_factors

    # Changes based on inspection:
    y[0] = 1  # Angle 0 (x[0]) has no error
    x.append(360) # As 360 == 0 degrees
    y.append(1)   # Error factor for this is also 1

    error_model = numpy.poly1d(numpy.polyfit(x, y, 4))
    print("\nError Model Coefficients:")
    print(error_model.coef)

    polynomial_file = output_file = open('mag_polynomal.txt', 'a')
    polynomial_file.write(str(error_model.coef).replace("\n","  "))  # remove weird newlines.
    polynomial_file.write("\n")

    output_file.close()
    polynomial_file.close()

    input("\nHit enter to start streaming Magnometer data: ")
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
