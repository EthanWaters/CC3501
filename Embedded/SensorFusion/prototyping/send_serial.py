import serial
import time
import sys

ser = serial.Serial('/dev/serial0')  # open serial port

# print(sys.argv)

if len(sys.argv) == 1:
    # No arguments:
    ser.write("This is a test\n".encode())
    print("Default Test")

elif len(sys.argv) == 2:
    arg = sys.argv[1]
    if str(arg) == "inf":
        # loop
        counter = 0
        while 1:
            # msg = ("Test: " + str(counter)).encode()
            # msg = [-19.45, 117.21, 74.1]
            msg = "-19.45, 117.21, 74.1\n"
            ser.write(msg.encode())
            print(msg.encode())
            counter += 1
            time.sleep(0.2)
    elif arg == "num":
        ser.write(13)
        # ser.write(3.1415)
        print("number")
    elif arg == "gps":
        ser.write("-19.12355, 129.54366, 76.1\n".encode())
        print("GPS Test")
    else:
        ser.write(arg.encode())
        print(arg.encode())

elif len(sys.argv) >= 3:
    for arg in sys.argv[1:]:
        ser.write(arg.encode())
        print(arg.encode())
        time.sleep(1)

