# send to pi
#!/usr/bin/env python

import pigpio

motor_pin = 0    # set
steering_pin = 0 # set
port = "8888"        # port of RPI on localhost


def initialize_pi():
   pi = pigpio.pi(port=port)
   if pi.connected:
      return pi
   else:
      print("Pi is not connected")
   
def stop_pi(pi):
   pi.stop()

def set_motor(pi,cv):
   pi.set_servo_pulsewidth(motor_pin, cv)

def set_steering(pi,cv):
   pi.set_servo_pulsewidth(steering_pin, cv)

def set_pwm(pi,pin, cv):
   pi.set_servo_pulsewidth(pin, cv)

