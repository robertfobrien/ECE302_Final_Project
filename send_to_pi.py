# send to pi
#!/usr/bin/env python
import serial

motor_pin = 0    # set
steering_pin = 0 # set
port = "22"        # port of RPI on localhost

def send_to_pi(ser, data):
    message = (str)(data) + "\n"
    ser.write(message.encode())
