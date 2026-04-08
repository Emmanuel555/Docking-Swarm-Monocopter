import pygame
import time

if __name__ == '__main__':

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    time_begin = time.time()
    time_end = time_begin + 1000
    time_last = -1

    while time_end > time.time():
        abs_time = time.time() - time_begin
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        joystick = pygame.joystick.Joystick(0)

        joystick.init()

        a0 = joystick.get_axis(0)  # x axis right hand <- -> ROLL
        a1 = -joystick.get_axis(1)  # y axis right hand up down PITCH
        a2 = joystick.get_axis(2)  # thrust up down COLLECTIVE
        a3 = -joystick.get_axis(3) # YAW

        L1 = joystick.get_button(0)  # default = 0
        R1 = joystick.get_button(1)  # default = 0

        
        lowest = 0.9
        highest = -1
        range_js = -(highest - lowest)
        range_motor = 65535

        rate = range_motor / range_js

        #print("controller buttons", "roll: ", a0, "pitch: ", a1, "collective: ", a2, "yaw: ", a3)
        print (L1, R1)
        #print('abs_time', abs_time)
        #print("number of buttons on controller: ", joystick.get_numbuttons())

        time_diff = time.time() - time_last
        time_last = time.time()
        time.sleep(0.05)
        #print('time_diff', time_diff)

        conPad = int((a2 - highest) * rate)

        print(conPad)

