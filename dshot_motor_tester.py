import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time
import threading

import Mocap
import DataSave
import Data_process

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

import monoco_att_ctrl
import trajectory_generator
import timeit
from pynput import keyboard

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


# robot address
# Change uris and sequences according to your setup

# monoco radio 1
URI1 = 'radio://0/80/2M/E7E7E7E705'


uris = {
    URI1,
}


def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
    }
    return seq_args


def swarm_direction_exe(direction):
    direction_args = {
        URI1: [direction[0]],  # 1 = reverse, 0 is forward
    }
    return direction_args


def swarm_logging(swarm_log):
    seq_args = {
        URI1: [swarm_log[0]],
    }
    return seq_args


def init_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.1)

        # initialise the controller
        cf.param.add_update_callback(group='stabilizer', name='estimator',
                                 cb=param_stab_est_callback)
        cf.param.set_value('stabilizer.controller', '3')  # 3: INDI controller
        time.sleep(0.1)

        #cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) # roll, pitch, yawrate, thrust 
        cf.commander.send_position_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation monoco....set ur sticks to mid and down")
        time.sleep(0.1)
    except Exception as e:
        print("Initialisation error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.commander.send_position_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3]))  
        #print('arming w thrust val....', cmds[3])
    except Exception as e:
        print("swarming error: ", e)


def transmitter_calibration(joystick):
    lowest = 0.97
    highest = -1
    range_js = -(highest - lowest)
    range_motor = 65535
    rate = range_motor / range_js
    a0 = joystick.get_axis(0)  # x axis right hand <- ->
    a1 = joystick.get_axis(1)  # y axis right hand up down
    a2 = joystick.get_axis(2)  # thrust
    trigger = round(joystick.get_axis(3))

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
    manual_thrust = conPad*enable*button0

    if cmd != 0:
        cmd = cmd/(65500/2)
    #print(f"Joystick_axes available: {joystick.get_numaxes()}")

    return (cmd, button0, button1, a0, a1, enable, conPad, button2, manual_thrust, trigger)


def p_control_input(linear_pos,kp,kv,ki,ref_pos,dt):
    """ position control """
    # error
    control_x = kp[0]*(ref_pos[0] - linear_pos[0]) - kv[0]*(linear_pos[3]) + ki[0]*(ref_pos[0] - linear_pos[0])*dt
    control_y = kp[1]*(ref_pos[1] - linear_pos[1]) - kv[1]*(linear_pos[4]) + ki[1]*(ref_pos[1] - linear_pos[1])*dt
    control_z = 1.0

    cmd = np.array([control_x, control_y, control_z])  # roll, pitch, yawrate, thrust
    return (cmd) 


def ref_manual_ctrl(a0,a1,alt):
    control_x = a0
    control_y = a1
    #control_z = 1.0
    control_z = alt # to include in MPC?

    #cmd = np.array([d_x_pad, d_y_pad, d_z_pad])  # roll, pitch, yawrate, thrust
    cmd = np.array([control_x, control_y, control_z])  # roll, pitch, yawrate, thrust
    return (cmd) 
    

def att_manual_ctrl(a0,a1,af):
    """ pad_x = a0
    pad_y = a1
    trigger = math.sqrt(pad_x ** 2 + pad_y ** 2)

    # direction of pad
    pad_dir = math.atan2(pad_y, pad_x)

    k = math.cos(af * math.pi / 180) # aggression factor means lowering the number
    d_x_pad = math.cos(pad_dir) * k #r
    d_y_pad = math.sin(pad_dir) * k #p
    d_z_pad = math.sqrt(1 - (d_x_pad ** 2 + d_y_pad ** 2)) """

    # to test
    #control_x = -0.02
    #control_y = -0.02

    control_x = a0
    control_y = a1
    #control_z = math.sqrt(1 - (control_x ** 2 + control_y ** 2))
    control_z = 1.0

    #cmd = np.array([d_x_pad, d_y_pad, d_z_pad])  # roll, pitch, yawrate, thrust
    cmd = np.array([control_x, control_y, control_z])  # roll, pitch, yawrate, thrust
    return (cmd) 

    
def logging_config():
    # Create the log config for the position
    lg_stab = LogConfig(name='gyro_tracking', period_in_ms=10) # limited to 100 hz, period = 10/1000
    #lg_stab.add_variable('gyro.x', 'float') # deg/s
    #lg_stab.add_variable('gyro.y', 'float')
    #lg_stab.add_variable('gyro.z', 'float')
    
    # attitude rate
    lg_stab.add_variable('ctrlINDI.Omega_f_p', 'float') # rad/s
    lg_stab.add_variable('ctrlINDI.Omega_f_q', 'float')
    lg_stab.add_variable('ctrlINDI.Omega_f_r', 'float')
    
    # attitude rate rate
    lg_stab.add_variable('ctrlINDI.rate_d_roll', 'float') # rad/s^2
    lg_stab.add_variable('ctrlINDI.rate_d_pitch', 'float')
    lg_stab.add_variable('ctrlINDI.rate_d_yaw', 'float')

    return (lg_stab)


def param_stab_est_callback(name, value):
    print('The crazyflie monocopter has parameter ' + name + ' set at number: ' + value)


def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    #gyro_x = data.get('gyro.x') # roll float
    #gyro_y = data.get('gyro.y') # pitch
    #gyro_z = data.get('gyro.z') # yaw

    # attitude rate (rad/s)
    omega_roll = data.get('ctrlINDI.Omega_f_p') # r 
    omega_pitch = data.get('ctrlINDI.Omega_f_q') # p
    omega_yaw = data.get('ctrlINDI.Omega_f_r') # y

    # attitude rate rate (rad/s^2)
    omega_roll_dot = data.get('ctrlINDI.rate_d_roll', 'float') # r 
    omega_pitch_dot = data.get('ctrlINDI.rate_d_pitch', 'float') # p
    omega_yaw_dot = data.get('ctrlINDI.rate_d_yaw', 'float') # y
    print('omega_roll_dot:', omega_roll_dot)


def log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    #time.sleep(5)
    #logconf.stop()


def direction_change_thread(scf, new_direction):
    global direction_changing, last_direction
    direction_changing = True

    """
    direction: 0 = forward, 1 = reverse
    """
    cf = scf.cf
    
    for param in cf.param.toc.toc:
        print(param)

    count = 0
    for group in cf.param.toc.toc:
        count += len(cf.param.toc.toc[group])
    print(f"Total params: {count}")

    try:
        cf = scf.cf
        cf.param.set_value('motorDir.direction', str(new_direction))
        cf.param.set_value('motorDir.trigger', '1')
        # wait for firmware to signal done
        while cf.param.get_value('motorDir.done') == '0':
            time.sleep(0.01)
    
        last_direction = new_direction
        direction_changing = False
        print(f"Motor direction changed to: {'reverse' if new_direction else 'forward'}")
    except Exception as e:
        print("Direction change error: ", e)


if __name__ == '__main__':
                                  
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

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
        joystick.get_axis(4)
        joystick.get_axis(5)
        joystick.get_axis(6)
        time.sleep(0.01)
    #print("Ready!")

    done = False
    controllerEnable = False
    pad_speed = 1

    time_last = 0
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 
    loop_counter = 0

    direction_changing = False
    last_direction = 0

    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # set init motor cmd to 0 0 0 0 (disarm)
        cmd_att = np.array([cmd_att_startup])
        current_direction = np.array([last_direction]) # default direction
        seq_dir = swarm_direction_exe(current_direction)
        swarm.parallel(direction_change_thread, args_dict=seq_dir)
        data_log = logging_config()
        swarm_log = np.array([data_log])
        seq_args_log = swarm_logging(swarm_log)
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)
        #swarm.parallel(log_async, args_dict=seq_args_log) # only can log up to six items at a time

        try:
            #while time_end > time.time():
            while True:
                pygame.event.pump()  # lightweight, just processes events without blocking
                start = timeit.default_timer() 
                abs_time = time.time() - time_start
                
                # time difference needed to calculate velocity
                dt = time.time() - time_last  #  time step/period
                time_last = time.time()

                # update from transmitter
                tx_cmds = transmitter_calibration(joystick)  # get the joystick commands
                manual_alt = tx_cmds[0]  # thrust command
                button0 = tx_cmds[1]
                button1 = tx_cmds[2]
                enable = tx_cmds[5]
                conPad = tx_cmds[6]
                button2 = tx_cmds[7]
                manual_thrust = tx_cmds[8]
                current_direction = tx_cmds[9]

                a0 = tx_cmds[3] # x     
                a1 = tx_cmds[4] # y

                # motor output
                motor_cmd = int(manual_thrust)


                # direction trigger
                if current_direction != last_direction and not direction_changing:
                    current_direction = np.array([current_direction])
                    seq_dir = swarm_direction_exe(current_direction)
                    swarm.parallel(direction_change_thread, args_dict=seq_dir)


                # motor saturation - manual thrust
                if motor_cmd > 65500:
                    motor_cmd = 65500
                elif motor_cmd < 10:
                    motor_cmd = 10

                final_cmd = np.array([motor_cmd, motor_cmd, motor_cmd, motor_cmd]) # e.g                
                final_cmd = np.array([final_cmd])
                seq_args = swarm_exe(final_cmd)
                swarm.parallel(arm_throttle, args_dict=seq_args)

                if loop_counter % 10 == 0:
                    print(f"Current direction: {current_direction}, Motor cmd: {motor_cmd}, Thrust: {manual_thrust}, X: {a0}, Y: {a1}, Enable: {enable}, Button0: {button0}, Button1: {button1}, ConPad: {conPad}, Button2: {button2}")     
        
                loop_counter += 1 

                #time.sleep(0.001) 


        except KeyboardInterrupt:  
           print ("motor test ended...")
            
                    