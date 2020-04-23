#!/usr/bin/env python


import Jetson.GPIO as GPIO
import time

# for 1st Motor on ENA
DIR = 31
PWM = 33


# set pin numbers to the board's
GPIO.setmode(GPIO.BOARD)

# initialize 

GPIO.setup(DIR, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PWM, GPIO.OUT, initial= 0)
time.sleep(1)

# Forward
speed = 5
while True:
	GPIO.output(DIR, GPIO.HIGH)
	GPIO.output(PWM, speed)

# Sleep
GPIO.output(DIR, GPIO.HIGH)
GPIO.output(PWM, 0)
time.sleep(1)



GPIO.cleanup()

