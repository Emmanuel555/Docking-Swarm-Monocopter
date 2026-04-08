import time
import pygame
import numpy as np
import timeit
import math
import numpy.linalg as la
from pyrr import quaternion


def transmitter_calibration():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    lowest = 0.97
    highest = -1
    range_js = -(highest - lowest)
    range_motor = 65535
    rate = range_motor / range_js
    a0 = joystick.get_axis(0)  # x axis right hand <- ->
    a1 = joystick.get_axis(1)  # y axis right hand up down
    a2 = joystick.get_axis(2)  # thrust
    a3 = joystick.get_axis(3)

    button0 = round(joystick.get_axis(5))
    button1 = round(joystick.get_axis(6))
    button2 = round(joystick.get_axis(4))
    
    # thrust from control pad
    conPad = int((a2 - highest) * rate)

    # throttle joystick saturation
    if conPad < 10:
        conPad = 10
    if conPad > 65500:
        conPad = 65500

    # takeoff sign
    if conPad < 2000:
        enable = 0
    else:
        enable = 1

    cmd = conPad*enable*button0 # thrust
    if cmd != 0:
        cmd = cmd/(65500/2)
    #print(f"Joystick_axes available: {joystick.get_numaxes()}")

    return (cmd, button0, button1, a0, a1, enable, conPad, button2)


if __name__ == '__main__':

    time_start = time.time()
    time_end = time_start + 1000
    last_time = time.time()
    count = 0

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1
    t_horizon = 1/2
    now = time.time()
   
    #try:
    while True:
        #if count > 0:
        #    print(f"update rate in s: {dt+sleep_time}")
        start = timeit.default_timer() 
        dt = time.time() - now
        now = time.time()  
        abs_time = now - time_start

        # dk why need this for python 3.10
        # joystick = pygame.joystick.Joystick(0) # added here to speed up loop

        ## update from transmitter
        tx_cmds = transmitter_calibration()  # get the joystick commands
        manual_thrust = tx_cmds[0]  # thrust command
        button0 = tx_cmds[1]
        button1 = tx_cmds[2]
        enable = tx_cmds[5]
        conPad = tx_cmds[6]
        button2 = tx_cmds[7]

        a0 = tx_cmds[3]     
        a1 = tx_cmds[4]
        #dt = now - last_time
        #last_time = now

        #if count % 0.3 == 0:
        print(f"Thrust: {manual_thrust}, X: {a0}, Y: {a1}, Enable: {enable}, Button0: {button0}, Button1: {button1}, ConPad: {conPad}, Button2: {button2}")     
        print(f"ConPad: {1000+((conPad/65500)*1000)}")
        #how fast the loop goes at
        #print(f"dt: {dt}")
        #sleep_time = max(0.0, t_horizon - (time.time() - now))
        #print (f"sleep_time: {sleep_time}")
        #time.sleep(sleep_time)
        count += 1

    #except KeyboardInterrupt: 
        #print("Exiting the program...")

        

