#https://towardsdatascience.com/sending-data-from-a-raspberry-pi-sensor-unit-over-serial-bluetooth-f9063f3447af

#give the path to the bluetooth serial port for the RPI
def open_comms(path):
     with open(path, 'w', 1) as f:
        return f

#set motor compare value for PWM
def set_motor(value, f):
    f.write('m;%f\n', value)

#set steering PWM 
def set_steering(value, f):
    f.write('s;%f\n', value)