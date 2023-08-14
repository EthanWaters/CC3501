#!/usr/bin/env python

import time
import datetime

import serial
from ublox_gps import UbloxGps

with serial.Serial('/dev/ttyS0', baudrate=38400, timeout=1) as port:
    gps = UbloxGps(port)

    try:
        print("GPS Process Start:")
        while True:
            geo = gps.geo_coords()

            unixtime = datetime.datetime.now().timestamp() * 1000
            result = "$[{:.0f}] [GPS] <[{:.6f}, {:.6f}, {:.4f}]>".format(unixtime, geo.lat, geo.lon, geo.headMot)
            print(result)

    except (ValueError, IOError) as err:
        print(err)

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt")

