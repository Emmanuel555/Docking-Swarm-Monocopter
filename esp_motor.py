import socket
import threading
import pygame
import time
import timeit
import serial

ESP32_IP = "192.168.1.108"
ESP32_PORT = 5005

PC_IP = "0.0.0.0"   # listen on all interfaces
PC_PORT = 5005
esp32_addr = None

# ESPNOW
SERIAL_PORT = '/dev/ttyACM0'  # change to your port, Windows would be 'COM3' etc
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)


""" sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PC_IP, PC_PORT))
print(f"PC UDP listening on {PC_IP}:{PC_PORT}") """


def transmitter_calibration(joystick):
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


def receive_loop():
    global esp32_addr
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode(errors="ignore").strip()
        esp32_addr = addr
        print(f"\nFrom ESP32/Teensy {addr}: {msg}")


if __name__ == '__main__':        

    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    recv_thread.start()

    time_start = time.time()
    time_end = time_start + 1000
    last_time = time.time()
    count = 0

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # warmup - run 50 iterations before starting
    #print("Warming up...")
    for _ in range(50):
        pygame.event.pump()
        joystick.get_axis(0)
        joystick.get_axis(1)
        joystick.get_axis(2)
        joystick.get_axis(3)
        time.sleep(0.01)
    #print("Ready!")

    done = False
    controllerEnable = False
    pad_speed = 1
    t_horizon = 1/2
    now = time.time()

    while True:
        pygame.event.pump()  # lightweight, just processes events without blocking
        start = timeit.default_timer() 
        dt = time.time() - now
        now = time.time()  
        abs_time = now - time_start

        # dk why need this for python 3.10
        # joystick = pygame.joystick.Joystick(0) # added here to speed up loop

        ## update from transmitter
        tx_cmds = transmitter_calibration(joystick)  # get the joystick commands
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
        #print(f"Thrust: {manual_thrust}, X: {a0}, Y: {a1}, Enable: {enable}, Button0: {button0}, Button1: {button1}, ConPad: {conPad}, Button2: {button2}")     
        
        conPad = 1000+((conPad/65500)*1000) # for PWM
        #conPad = conPad/65500 # for dshot
        #print(f"ConPad: {conPad}")

        #how fast the loop goes at
        #print(f"dt: {dt}")
        #sleep_time = max(0.0, t_horizon - (time.time() - now))
        #print (f"sleep_time: {sleep_time}")
        #time.sleep(sleep_time)
        count += 1

        try:
            ##pwm
            pwm = int(conPad)
            pwm = max(1000, min(2000, pwm)) # pwm
            #pwm = int(1200)
            print(f"PWM: {pwm}, Update rate: {1/dt:.2f} Hz")
            ser.write(pwm.to_bytes(2, byteorder='little'))
            #ser.write((str(pwm) + "\n").encode())
            #sock.sendto((str(pwm) + "\n").encode(), (ESP32_IP, ESP32_PORT)) # hardcoded
            #sock.sendto((str(pwm) + "\n").encode(), esp32_addr) # auto

            ##dshot
            """ dshot = float(conPad)
            dshot = round(dshot,3)
            dshot = max(0, min(1.0, dshot)) # dshot
            print (str(button0) + "," + str(dshot))
            sock.sendto((str(button0) + "," + str(dshot) + "\n").encode(), (ESP32_IP, ESP32_PORT)) # hardcoded
             """
            #sock.sendto((str(pwm) + "\n").encode(), esp32_addr) # auto


        except ValueError:
            print("Invalid input, enter a number between 1000-2000")

        time.sleep(1/250) 