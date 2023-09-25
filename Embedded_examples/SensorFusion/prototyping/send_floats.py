from random import random
import serial
import time
import sys
import struct
import math


def float_to_bytes(float_array):
    data_bytearray = bytearray()

    data_bytearray.extend('A'.encode())

    for float in float_array:
        data_bytearray.extend(bytearray(struct.pack("f", float)))

    data_bytearray.extend('Z'.encode())

    return data_bytearray # [ b for b in my_bytearray ]


data = [120.1, 30.8, 167.1]
my_bytearray = float_to_bytes(data)
# print([ b for b in ba ])

ser = serial.Serial('/dev/serial0',
                    baudrate=9600, 
                    timeout=0,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
)  # open serial port

if len(sys.argv) == 1:
    # No arguments:
    for byt in my_bytearray:
        ser.write(byt)
        print(byt, end=' ')
    
    print("")

elif len(sys.argv) == 2:
    if sys.argv[1] == "loop":
        # LOOP:
        count = 0
        while True:
            count += 1
            # for byt in my_bytearray:
            #     ser.write(byt)
            #     print(byt, end=' ')

            # data = [120.1, 30.8, 167.1]
            data = [math.cos(count / 100.0), math.sin(count / 100.0), 0.2 * math.sin(count / 200.0)]
            my_bytearray = float_to_bytes(data)

            ser.write(my_bytearray)
            print([b for b in my_bytearray], end='')
            print(f"    {time.time():.3f}")
            time.sleep(0.05)



