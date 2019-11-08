import serial
ser = serial.Serial()
ser.baudrate = 115200
ser.port = "/dev/serial/by-id/usb"
ser.open()
ser.readline() [20:26]
ser.close()