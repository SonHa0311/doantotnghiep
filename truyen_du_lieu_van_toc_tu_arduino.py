#!/usr/bin/env python3
import serial
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.5)
    ser.reset_input_buffer()
    while True:
        if ser.in_waiting > 0:
           line = ser.readline().decode('latin-1').rstrip() # hoac cho decode doi thanh 'latin-1' hoac latin-1
           print("Real velocity = ",line)