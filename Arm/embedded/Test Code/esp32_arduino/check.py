import serial

# Specify the COM port name and other settings
port_name = 'COM9'  # Replace 'COMX' with the actual COM port name, e.g., 'COM1' or '/dev/ttyUSB0'
baud_rate = 115200  # Set the baud rate to match the configuration of the connected device

try:
    # Open the COM port
    ser = serial.Serial(port_name, baudrate=baud_rate)
    if ser.is_open:
        print(f"Serial port {port_name} opened successfully.")
    
    # You can now read from or write to the COM port
    
    # Example: Read data from the COM port
    data = ser.readline()
    print(f"Received data: {data.decode('utf-8')}")

    # Example: Write data to the COM port
    ser.write(b'Hello, Serial Port!\n')

    # Don't forget to close the COM port when you're done
    ser.close()
    print(f"Serial port {port_name} closed.")
except serial.SerialException as e:
    print(f"Error opening the serial port: {e}")
