import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time

import Mocap
import DataSave
import Data_process

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

#import long_monoco_att_ctrl
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
URI1 = 'radio://0/80/2M/E7E7E7E701'


uris = {
    URI1,
}


def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
    }
    return seq_args


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


def transmitter_calibration():
    # where hand control comes
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

    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)

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

    cmd = conPad*enable*button0

    return (cmd, button0, button1, a0, a1, enable)


def p_control_input(linear_pos,kp,kv,ki,ref_pos,dt):
    """ position control """
    # error
    control_x = kp[0]*(ref_pos[0] - linear_pos[0]) - kv[0]*(linear_pos[3]) + ki[0]*(ref_pos[0] - linear_pos[0])*dt
    control_y = kp[1]*(ref_pos[1] - linear_pos[1]) - kv[1]*(linear_pos[4]) + ki[1]*(ref_pos[1] - linear_pos[1])*dt
    control_z = 1.0

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


if __name__ == '__main__':


    # Setup keyboard listener
    stage = 'hover'
    def on_press(key):
        global stage
        try:
            if key.char == 't':  # Check if 't' is pressed
                stage = 'trajectory on'
            if key.char == 'h':  # Check if 'h' is pressed
                stage = 'hover'
            if key.char == 'l':  # Check if 'l' is pressed
                stage = 'land'          
        except AttributeError:
            pass  # Ignore special keys

    # Start listening for key presses
    listener = keyboard.Listener(on_press=on_press)
    listener.start()


    data_receiver_sender = Mocap.Udp()
    max_sample_rate = 250 # 360 at 65
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    #data_saver = DataSave.SaveData('Data_time',
    #                               'Monocopter_XYZ','ref_position','rmse_num_xyz','final_rmse','ref_msg','status','cmd','tpp_angle')

    #data_saver = DataSave.SaveData('Data_time',
    #                               'Monocopter_XYZ','motor_cmd','ref_position','tpp_roll','tpp_pitch','body_yaw_deg','tpp_omega','tpp_omega_dot','body_angle_roll',
    #                               'rmse_num_xyz','att_error','att_rate_error','att_raterate_error','yawrate')    

    data_saver = DataSave.SaveData('Data_time',
                                   'Monocopter_XYZ','rotational_state_vector','motor_cmd','ref_position','ref_velocity','motor_actual_cmd','cmd_bod_acc','yawrate') 
                
                                   
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1


    # Status
    status = "pending for state estimation"


    # rmse terms
    x_error = 0.0
    y_error = 0.0
    z_error = 0.0
    rmse_num_x = 0.0
    final_rmse_x = 0.0
    rmse_num_y = 0.0
    final_rmse_y = 0.0
    rmse_num_z = 0.0
    final_rmse_z = 0.0


    # must always start from the ground
    # loop rates
    loop_counter = 1
    att_loop = 1
    pid_loop = 1
    rate_loop = 1
    
    
    time_last = 0
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 


    # collective z 
    kpz = 5000 
    kdz = 6000 
    kiz = 1200 # | 128

    apz = 320000 
    adz = 30000 
    aiz = 1000 # | 128


    # cyclic xyz (position)
    kp = [1.0,1.0,0.0] # 0.04
    kd = [0.0005,0.0005,0.0] # not in use
    ki = [10.0,10.0,0.0] 


    # cyclic xyz (velocity)
    kvp = [0.0001,0.0001,0.0] 
    

    # cyclic xy (attitude) - heuristic gains thus far
    ka = [6000, 6000]  # 6000
    kr = [5.0, 5.0] # 10
    krr = [1.0, 1.0] # 1.0
   

    # physical params
    wing_radius = 200/1000 # change to 700 next round
    chord_length = 0.12
    mass = 1000
    cl = 0.5
    cd = 0.052
    J = np.array([1/100000,1/100000,1/1000000]) # moment of inertia
    

    monoco = monoco_att_ctrl.att_ctrl(kp, kd, ki, kvp, ka, kr, krr)
    monoco.physical_params(wing_radius, chord_length, mass, cl, cd, J) 


     # Initialize references
    ref_pos_circle = np.array([0.0,0.0,0.0])
    ref_pos = np.array([1.0,0.0,1.0]) # 0,0 fked up for some reason
    land_pos = np.array([0.0,0.0,0.6])
    x_hover_offset = ref_pos[0]
    y_hover_offset = ref_pos[1]
    z_hover_offset = ref_pos[2]
    x_land_offset = land_pos[0]
    y_land_offset = land_pos[1]
    z_land_offset = land_pos[2]
    ref_pos_z = 0.0
    ref_vel = np.array([0.0,0.0,0.0])
    ref_acc = np.array([0.0,0.0,0.0])
    ref_jerk = np.array([0.0,0.0,0.0])
    ref_snap = np.array([0.0,0.0,0.0])
    ref_msg = "havent computed yet" 


    # reference offset for xyz
    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0

    # flatness option
    flatness_option = 1  # this is a must!
    amplitude = 1  

    manual_cyclic = np.array([0.0, 0.0, 0.0]) 
    auto_cyclic = np.array([0.0, 0.0, 0.0]) 


    # circle parameters
    radius = 0.75 # 0.5 original was 1.0
    speedX = 10.0 # 0.5 m/s the best thus far
    laps = 3
    leminiscate_laps = 4
    leminiscate_radius = 1.5
    helix_laps = 9
    alt = ref_pos[2]
    elevated_alt = 1.0
    reverse_cw = 0 # 1 for clockwise, 0 for counterclockwise


    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()
    ## traj generator for min snap circle, ####### pre computed points
    ## 2 pt line
    #chosen_traj = "_2_pt_line_"
    #pva,num_pts = traj_gen.two_pt_line(speedX, max_sample_rate/pid_loop, alt)
    ## circle
    #chosen_traj = "_circle_"
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop, laps, reverse_cw, alt) # mechanical limit for monocopter is 0.5m/s
    ## lemniscate
    #chosen_traj = "_lemniscate_"
    #pva,num_pts = traj_gen.lemniscate(x_offset, y_offset, leminiscate_laps, leminiscate_radius, max_sample_rate/pid_loop, reverse_cw, speedX, alt)
    ## helix
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_helix_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop,helix_laps,reverse_cw,alt)
    ## elevated circle
    chosen_traj = "_elevated_circle_"
    pva,num_pts = traj_gen.compute_jerk_snap_9pt_elevated_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop,laps,reverse_cw,elevated_alt)

    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
        cmd_att = np.array([cmd_att_startup])
        data_log = logging_config()
        swarm_log = np.array([data_log])
        seq_args_log = swarm_logging(swarm_log)
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)
        #swarm.parallel(log_async, args_dict=seq_args_log) # only can log up to six items at a time

        try:
            #while time_end > time.time():
            while True:
                start = timeit.default_timer() 
                abs_time = time.time() - time_start

                # require data from Mocap
                data = data_receiver_sender.get_data()

                # data unpack
                data_processor.data_unpack_filtered(data)

                # processed tpp data/feedback
                # rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_finite_diff()
                rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
                pos_raw = data_processor.raw_data
                linear_state_vector = data_processor.pos_vel_acc_filtered()
                body_pitch = data_processor.body_pitch
                tpp_angle = data_processor.tpp
                tpp_omega = data_processor.Omega
                tpp_omega_dot = data_processor.Omega_dot
                body_yaw = data_processor.yaw
                yawrate = data_processor.get_yawrate()
                # tpp_omega = data_processor.Omega
                # tpp_omega_dot = data_processor.Omega_dot
                tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()
                bod_angle_roll = data_processor.body_angle_roll
                # time difference needed to calculate velocity
                dt = time.time() - time_last  #  time step/period
                time_last = time.time()

                # update positions etc.
                monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)

                tx_cmds = transmitter_calibration()  # get the joystick commands
                manual_thrust = tx_cmds[0]  # thrust command
                button0 = tx_cmds[1]
                button1 = tx_cmds[2]
                enable = tx_cmds[5]

                a0 = tx_cmds[3]     
                a1 = tx_cmds[4]
                af = 80 # aggression factor


                # update references for PID position loop
                if loop_counter % pid_loop == 0:
                

                    # update references for PID loop 
                    manual_cyclic = att_manual_ctrl(a0, a1, af) # manual position control 
                    
                    if button1 == 1:

                        ## hovering test
                        if stage == 'hover':
                            ref = traj_gen.hover_test(x_hover_offset,y_hover_offset,z_hover_offset)
                            hovering_ff = np.array([0.0, 0.0, 0.0])
                            ref_pos = ref[0]
                            ref_pos_z = ref_pos[2]
                            ref_vel = hovering_ff
                            ref_acc = hovering_ff
                            ref_jerk = hovering_ff
                            ref_snap = hovering_ff
                            ref_msg = ref[1]
                            count = 0


                        ## trajectory inputs
                        if stage == 'trajectory on':
                            ref_derivatives = traj_gen.jerk_snap_circle(pva,num_pts,count,alt)
                            ref_pos = ref_derivatives[0]
                            ref_pos_z = ref_pos[2]
                            ref_vel = ref_derivatives[1]
                            ref_acc = ref_derivatives[2]
                            ref_jerk = ref_derivatives[3]
                            ref_snap = ref_derivatives[4]
                            ref_msg = ref_derivatives[5]  
                            # compute bem thrust
                            monoco.compute_bem_wo_rps(body_pitch) 
                            count += 1 


                        ## landing 
                        if stage == 'land':
                            ref = traj_gen.hover_test(x_land_offset,y_land_offset,z_land_offset)
                            hovering_ff = np.array([0.0, 0.0, 0.0])
                            ref_pos = ref[0]
                            ref_pos_z = ref_pos[2]
                            ref_vel = hovering_ff
                            ref_acc = hovering_ff
                            ref_jerk = hovering_ff
                            ref_snap = hovering_ff
                            ref_msg = ref[1]
                            count = 0

                        # ff references
                        monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap,ref_pos_z)
                        # p control
                        auto_cyclic = p_control_input(linear_state_vector, kp, kvp, ki, ref_pos, sample_time) # auto position control
                        monoco.p_control_input_manual(auto_cyclic)
                        monoco.v_control_input()
                    else:
                        monoco.p_control_input_manual(manual_cyclic) # update the manual cyclic inputs
                        #monoco.v_control_input()

                    
                if loop_counter % att_loop == 0:

                    # get angle
                    cmd_att = monoco.get_angle()
                    att_error = monoco.attitude_error


                if loop_counter % rate_loop == 0:

                    # bod rates
                    monoco.get_body_rate(flatness_option)
                    att_rate_error = monoco.attitude_rate_error
                

                # collective thrust (alt hold)
                if button1 == 1:
                    z_controls = monoco.collective_thrust(apz,adz,aiz) # full z alt hold
                    collective_thrust = z_controls[0]*enable*button0
                else:    
                    z_controls = monoco.manual_collective_thrust(kpz,kdz,kiz,manual_thrust)
                    collective_thrust = z_controls[0]*enable*button0


                # from att ctrl
                cmd_bod_acc = monoco.CF_SAM_get_angles_and_thrust(enable,flatness_option)
                att_raterate_error = monoco.attitude_raterate_error
                cyclic = cmd_bod_acc[0] + cmd_bod_acc[1]


                # motor output
                motor_cmd = collective_thrust + int(cyclic)*button0


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
                    print('cmd and button commands: ', motor_cmd, button0, button1)
                    print(ref_msg) 
                    #print('tx commands: ', a0, a1)
                    #print('tpp_position', linear_state_vector[0], linear_state_vector[1], linear_state_vector[2])
                    print('altitude: ', linear_state_vector[2])
                    #print('manual_cyclic_xyz: ', auto_cyclic)
                    #print('p_cyclic_xyz: ', monoco.p_control_signal)
                    print('att_cmds: ', cmd_bod_acc)
                    print('monoco.rates comparison: ', monoco.cmd_bod_rates_final, monoco.ref_rates)
                    print('monoco.raterates comparison: ', monoco.cmd_bod_raterates_final, monoco.ref_raterates)
                    #print('yawrate: ', yawrate)

                    if dt > 0.0:
                        print('frequency (Hz) = ', 1/dt)
                        #print('time step: ', dt, 'abs time: ', abs_time)


                # control loop counter
                loop_counter += 1


                # collect data 
                if button1 == 1:
                    if stage == 'trajectory on':
                        x_error = ref_pos[0]-x_offset-linear_state_vector[0]
                        y_error = ref_pos[1]-y_offset-linear_state_vector[1]
                        z_error = ref_pos[2]-z_offset-linear_state_vector[2]
                        rmse_num_x = rmse_num_x + (x_error)**2
                        rmse_num_y = rmse_num_y + (y_error)**2
                        rmse_num_z = rmse_num_z + (x_error )**2
                        rmse_num = [x_error,y_error,z_error,rmse_num_x,rmse_num_y,rmse_num_z]
                        
                        #data_saver.add_item(abs_time,
                        #            linear_state_vector[0:3],motor_cmd,ref_pos,round((tpp_angle[0]*(180/np.pi)),3),round((tpp_angle[1]*(180/np.pi)),3),round(body_yaw*(180/np.pi),2),tpp_omega,tpp_omega_dot,bod_angle_roll,
                        #            rmse_num,att_error,att_rate_error,att_raterate_error,yawrate)    
                        
                        data_saver.add_item(abs_time,linear_state_vector[0:6],rotational_state_vector,motor_cmd,ref_pos,ref_vel,motor_cmd,cmd_bod_acc,yawrate) 

                    

        except KeyboardInterrupt:  
            # final rmse calculation
            status = "Emergency stop"
            ref_msg = "flight ended..."
            # final_rmse_x = math.sqrt(rmse_num_x/(count))
            # final_rmse_y = math.sqrt(rmse_num_y/(count))
            # final_rmse_z = math.sqrt(rmse_num_z/(count))
            # final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
            # rmse_num = [final_rmse_x, final_rmse_y, final_rmse_z]  

            #print('Emergency Stopped and final rmse produced: ', rmse_num )
            
monoco_name = 'long'

# save data
path = '/home/emmanuel/Monocopter-OCP/DFBC/fan_DFBC_' + monoco_name + chosen_traj + str(speedX*0.1) + '_ms'
data_saver.save_data(path)

