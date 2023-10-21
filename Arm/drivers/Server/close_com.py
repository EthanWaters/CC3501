import serial

# Open COM port
ser1 = serial.Serial('COM1', 9600)  # Replace 'COM18' with your COM port name

# Do some communication

# Close COM port
ser1.close()
