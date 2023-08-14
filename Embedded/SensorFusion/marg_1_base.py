from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman

from threading import Thread
import time

import smbus
import math
import struct
import numpy
from datetime import datetime

import serial
from ublox_gps import UbloxGps

from gpsfuser.gpsKalmanFilter import margKalmanFilter
from gpsfuser.gpsHelperMethods import margHelperMethods


def float_to_bytes(float_array):
    data_bytearray = bytearray()

    data_bytearray.extend('A'.encode())

    for float in float_array:
        data_bytearray.extend(bytearray(struct.pack("f", float)))

    data_bytearray.extend('Z'.encode())

    return data_bytearray


def map_yaw(yaw_raw):
    # Map yaw to 0-360 range:
    if yaw_raw < 0:
        yaw_pos = yaw_raw + 360
    else:
        yaw_pos = yaw_raw

    # error_model = numpy.poly1d([ 1.48837301e-07, -8.94532630e-05,  1.29375841e-02,  1.02891384])
    # error_model = numpy.poly1d([-8.84018876e-10,  7.67953538e-07, -2.16196333e-04,  1.92186193e-02,  1.08444277e+00])
    error_model = numpy.poly1d([-2.35450230e-10,  2.77331670e-07, -1.00195792e-04,  1.12039948e-02, 9.86449751e-01]) #AIMS

    yaw_map = (error_model(yaw_pos) * yaw_pos) % 360

    return [yaw_map, yaw_pos]


def get_gps(gps):
    '''This functions typically runs in a Thread as gps.get_coords() runs at 1 Hz.'''
    global gps_data
    global is_gps_updated
    global stop_threads  # Flag to stop gps updates.

    while not stop_threads:
        geo = gps.geo_coords()

        # print(f"{geo.lat:>.6f} {geo.lon:>.6f}")

        gps_data = {'lat': geo.lat, 'lon': geo.lon, 'alt': geo.height, 'vel_north': geo.velN / 1e3,
                    'vel_east': geo.velE / 1e3, 'vel_down': geo.velD / 1e3, 'vel_error': geo.sAcc / 1e3}

        is_gps_updated = True

    print("GPS Thread Exited.")


def main():
    '''Main Process'''

    # GPS Setup:
    global gps_data
    global is_gps_updated
    
    gps_visual_update_flag = False # For fusing the gps data.
    is_gps_updated = False # For visual feedback.
    gps_recieved_counter = 0 
    gps_required_for_start = 4


    gps_data = None
    last_gps_timestamp = 0

    port = serial.Serial('/dev/ttyS0', baudrate=38400, timeout=1)
    gps = UbloxGps(port)
    # UBX-NAV-PVT

    Thread(target=get_gps, args=(gps, )).start()

    print("Spooling IMU...")

    # IMU Setup:
    address = 0x68
    bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(bus, address)
    imu.begin()

    imu.loadCalibDataFromFile("/home/pi/SensorFusion/CalibrationFiles/TEST_CAL_1.json")

    imufusion = kalman.Kalman()
    previous_time = time.time()

    # Fusion setup:
    # Standart deviations:
    std_latlon = 2.0
    std_altitude = 3.518522417151836
    std_accel_north = 9.81 * 0.033436506994600976
    std_accel_east = 9.81 * 0.033436506994600976 # 0.05355371135598354
    std_accel_up = 9.81 * 0.2088683796078286  # ~ down

    # Create objects + do intial predict:
    helperObj = margHelperMethods()

    initial_gps_data = {'lat': -19.3154967, 'lon': 146.7862175, 'alt': 42,
                        'vel_north': 0, 'vel_east': 0, 'vel_down': 0, 'vel_error': 0}

    timestamp = time.time()

    obj_north = margKalmanFilter(
        helperObj.latToMtrs(initial_gps_data["lat"]),
        initial_gps_data["vel_north"], std_latlon,
        std_accel_north, timestamp)

    obj_east = margKalmanFilter(
        helperObj.lonToMtrs(initial_gps_data["lon"]),
        initial_gps_data["vel_east"], std_latlon,
        std_accel_east, timestamp)

    obj_up = margKalmanFilter(
        initial_gps_data["alt"],
        initial_gps_data["vel_down"] * -1.0, std_latlon, # Direction is reversed.
        std_accel_up, timestamp)

    # Save/send the data!
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d-%b-%Y (%H-%M-%S.%f)")
    output = open(f'output/output {timestampStr}.txt', 'a')

    # Loop variables:
    has_started = False
    # target_delay = 0.05
    # serial_loop_timer = time.time()
    
#     serial_out = serial.Serial(
#         port='/dev/serial0',
#         baudrate = 9600,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         bytesize=serial.EIGHTBITS,
#         timeout=1
#     )

    # Main Loop:
    while True:

        # IMU Data:
        imu.readSensor()
        imu.computeOrientation()

        # Calcuate change in time:
        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time

        accel_values = [imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]]

        imufusion.computeAndUpdateRollPitchYaw(accel_values[0], accel_values[1], accel_values[2], 
                                                imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], 
                                                imu.MagVals[0], imu.MagVals[1], imu.MagVals[2],
                                                dt)

        yaw_mapped, yaw_positive = map_yaw(imufusion.yaw)  # (Mapped Yaw, Positive Yaw)

        # Compute variables:
        yaw_mapped_radians = math.radians(yaw_mapped)

        # TODO: currently it assumes it's flat on the ground, change this.
        accel_north = -math.cos(yaw_mapped_radians) * imu.AccelVals[0] + math.sin(yaw_mapped_radians) * imu.AccelVals[1]
        accel_east = -math.sin(yaw_mapped_radians) * imu.AccelVals[0] + math.cos(yaw_mapped_radians) * imu.AccelVals[1]
        accel_down = accel_values[2] + 9.81 + 0.312

        # print(f"{accel_north: .2f} {accel_east: .2f} {accel_down: .2f}   {yaw_mapped: .2f}")
        # print(f"{imu.AccelVals[0]: .2f} {imu.AccelVals[1]: .2f} {imu.AccelVals[2]: .2f}   {yaw_mapped: .2f}")

        # Call Predict:
        timestamp = time.time()
        obj_north.predict(accel_north, timestamp)
        obj_east.predict(accel_east, timestamp)
        obj_up.predict(accel_down * -1, timestamp)  # reverse

        # GPS Data:
        if is_gps_updated:

            gps_recieved_counter += 1

            is_gps_updated = False # GPS result has been used.
            gps_visual_update_flag = True # Use this flag for visual indication.
            last_gps_timestamp = current_time

            defPosErr = 0.0

            # TODO: change this:
            gps_data["altitude_error"] = 2.32

            # Call the update function for all objects
            vel_north = -gps_data["vel_north"]
            latitude = obj_north.latToMtrs(gps_data["lat"])
            obj_north.update(latitude, vel_north, defPosErr,
                             gps_data["vel_error"])

            vel_east = gps_data["vel_east"]
            longitude = obj_east.lonToMtrs(gps_data["lon"])
            obj_east.update(longitude, vel_east, defPosErr,
                            gps_data["vel_error"])

            vel_up = gps_data["vel_down"] * -1.0  # reverse
            obj_up.update(gps_data["alt"], vel_up, gps_data["altitude_error"],
                          gps_data["vel_error"])

            # End GPS update.            

        # Get Predicted Values:
        predictedLatMtrs = obj_north.getPredictedPos()
        predictedLonMtrs = obj_east.getPredictedPos()
        predictedAlt = obj_up.getPredictedPos()

        if not has_started:
            has_started = True
            lat_origin_meters = predictedLatMtrs
            lon_origin_meters = predictedLonMtrs
        
        lat_delta_meters = predictedLatMtrs - lat_origin_meters
        lon_delta_meters = predictedLonMtrs - lon_origin_meters

        predictedLat, predictedLon = helperObj.mtrsToGeopoint(predictedLatMtrs, predictedLonMtrs)

        # The program should recieve a few GPS messages before displaying data to increase initial accuracy:
        if (gps_recieved_counter <= gps_required_for_start) & gps_visual_update_flag:
            # Has not recieved enough signals:
            print(f"Awaiting {gps_recieved_counter}/{gps_required_for_start} gps messages to start...")
            gps_visual_update_flag = False

        elif gps_recieved_counter > gps_required_for_start:
            # Ready to send:
            result_line = f"{time.strftime('%Y%m%d-%H%M%S')}, {lat_delta_meters:> 3.1f}, {lon_delta_meters:> 3.1f}, {predictedLat:>.6f}, {predictedLon:>.6f}, {yaw_mapped:> 6.1f}, {obj_north.getPredictedVel():> 4.1f}, {obj_east.getPredictedVel():> 4.1f}"
            print(result_line, end='')

            # Show GPS update status:
            if gps_visual_update_flag:
                gps_visual_update_flag = False
                print("\t\t[-:--s \tGPS RECIEVED]")
            else:
                print(f"\t\t[{current_time - last_gps_timestamp:.2f}s]")

            # Write it to file:
            output.write(result_line + "\n")

            #####################
            ### USE DATA HERE ###
            #####################
            
            # lat_delta_meters -> Meters from starting point in X axis (North-South)
            # lon_delta_meters -> Meters from starting point in Y axis (East-West)
            # yaw_mapped       -> Angle from North in the range 0 to 360.
                
            # # SEND DATA OVER SERIAL:
            # if (time.time() - serial_loop_timer) >= (target_delay):
            #     data_bytearray = float_to_bytes([lat_delta_meters, lon_delta_meters, yaw_mapped])
            #     serial_out.write(data_bytearray)
            #     serial_loop_timer = time.time()    

            # predictedVN = obj_north.getPredictedVel()
            # predictedVE = obj_east.getPredictedVel()
            # resultantV = math.hypot(predictedVE, predictedVN)

        time.sleep(0.01)


if __name__ == "__main__":
    global stop_threads
    stop_threads = False

    try:
        main()
    except KeyboardInterrupt:
        print('\nProgram Stopping...')
    finally:
        print("Ending Threads...")
        stop_threads = True
