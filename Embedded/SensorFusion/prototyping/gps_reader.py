#!/usr/bin/env python

import time
import datetime

import serial
from ublox_gps import UbloxGps

port = serial.Serial('/dev/ttyS0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

status = open('status', 'r')
output = open('output.txt', 'a')


def run():
    print("GPS Process Start:")
    try:
        while True:
            # Get status:
            status.seek(0)
            cmd = status.read()

            if cmd == "0":
                print("Process Stopped")
                break
            elif cmd == "p":
                print("Process Paused")
                time.sleep(5)
                continue
            elif cmd == "1":
                # Get GPS data and write to file:
                geo = gps.geo_coords()

                unixtime = datetime.datetime.now().timestamp() * 1000
                result = "$[{:.0f}] [GPS] <[{:.6f}, {:.6f}, {:.4f}]>".format(unixtime, geo.lat, geo.lon, geo.headMot)
                output.write(result + "\n")
                output.flush()
                print(result)

    except (ValueError, IOError) as err:
        print(err)

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt")

    finally:
        port.close()
        status.close()
        output.close()


if __name__ == '__main__':
    run()
