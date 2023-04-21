
#https://towardsdatascience.com/sending-data-from-a-raspberry-pi-sensor-unit-over-serial-bluetooth-f9063f3447af

def open_comms(path):
     with open(path, 'w', 1) as f:
        return f

def set_motor_PWM(value, f):
    f.write('(m;%f)', f)

def set_steering_PWM(value, f):
    f.write('(s;%f)', f)