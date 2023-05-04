# send to pi
#!/usr/bin/env python
import serial

def send_to_pi(ser, data):
    message = (str)(data) + "\n"
    ser.write(message.encode())
