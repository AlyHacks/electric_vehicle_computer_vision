import pigpio
import time
import math

gpio_pin = 14
pre_setRPM = 210_000
diameter = 3

pi = pigpio.pi()
pi.set_mode(gpio_pin, pigpio.OUTPUT)

def run(SECS):
    pi.write(gpio_pin, 1)
    time.sleep(SECS)
    pi.write(gpio_pin, 0)
    time.sleep(SECS)


def RPM(distance): #distance in inches
    dist_sec = (pre_setRPM/60)*(3*pi)
    needed_secs = distance/dist_sec
    return needed_secs

try:
    run(RPM(12))
    time.sleep(5)
    RPM(4)
except:
    KeyboardInterrupt()
    

pi.stop()


