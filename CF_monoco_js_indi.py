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

import monoco_att_ctrl
import trajectory_generator
import timeit
from pynput import keyboard

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


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


def init_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.1)
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation swarming....set ur sticks to mid and down")
        time.sleep(0.1)
    except Exception as e:
        print("Initialisation swarming error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        #print('arming w thrust val....', cmds[3])
    except Exception as e:
        print("swarming error: ", e)


if __name__ == '__main__':

    # Setup keyboard listener
    stage = 'hover'
    # def on_press(key):
    #     global stage
    #     try:
    #         if key.char == 'g':  # Check if 'g' is pressed
    #             stage = 'trajectory on'
    #     except AttributeError:
    #         pass  # Ignore special keys

    # # Start listening for key presses
    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()

    data_receiver_sender = Mocap.Udp()
    max_sample_rate = 360
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [64], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'Monocopter_XYZ','ref_position','rmse_num_xyz','final_rmse','ref_msg','status','cmd','tpp_angle','tpp_omega','tpp_omega_dot','velocity','z_control','des_thrust','ref_rates','ref_raterates','precession_rate','yawrate')
                      
                                   
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

  # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1
    # status
    status = "pending for state estimation"
    
    # reference offset for xyz
    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0

    # rmse terms
    rmse_num_x = 0.0
    final_rmse_x = 0.0
    rmse_num_y = 0.0
    final_rmse_y = 0.0
    rmse_num_z = 0.0
    final_rmse_z = 0.0

    # must always start from the ground
    # loop rates
    loop_counter = 1

    raterate_loop = 1 # 360/3 = 120hz

    rate_loop = raterate_loop * 2 # 120 hz, 2 is the best thus far
    #att_loop = rate_loop * 1.5 # same as rate loop, try next wk on pos hold
    att_loop = rate_loop * 5 # 10, 36 hz best so far
    #vel_loop = rate_loop * 5 # 8, 36 hz best so far
    #pid_loop = rate_loop * 5 # 10, 36 hz best so far, position
    pid_loop = 1

    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()

    # circle parameters
    radius = 0.8 # 0.5
    speedX = 2.0 # 2 still the best
    laps = 30
    helix_laps = 32
    reverse_cw = 0 # 1 for clockwise, 0 for counterclockwise
    alt = 1.5
    elevated_alt = 0.8

    # hover parameters
    x_hover_offset = 0.0
    y_hover_offset = 0.0
    
    ## traj generator for min snap circle, ####### pre computed points
    ## 2 pt line
    #pva,num_pts = traj_gen.two_pt_line(speedX, max_sample_rate/pid_loop, alt)
    # pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle(0, 0.5, 1)
    ## circle
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop, laps, reverse_cw, alt) # mechanical limit for monocopter is 0.5m/s
    ## elevated circle
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_elevated_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop, laps, reverse_cw, elevated_alt)
    ## helix
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_helix_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop,helix_laps,reverse_cw,alt)


    # collective z 
    kpz = 9.6 # 7.5, 30.5 |  9.6
    kdz = 5.0 # 2.5, 10.5 | 5.0
    kiz = 128 # | 128
    
    # cyclic xyz (position)
    kp = [0.8,0.8,0.0] # 0.45 - 1.5 * 0.1m/s 0.02     0.8
    kd = [0.03,0.03,0.0] # 0.2 - 1.5 * 0.1m/s 0.032  0.025
    ki = [0.0,0.0,0.0] # 0.0015   0.003

    # cyclic xyz (velocity) - monocopter doesnt like lol
    kvp = [0.0,0.0,0.0] 
    kvd = [0.0,0.0,0.0] 
    kvi = [0.0,0.0,0.0] 

    # cyclic xy (attitude)
    ka = [1.0, 1.0]  # 0.08 - 1.5 * 0.1m/s
    kad = [0.0, 0.0]
    kai = [0.0, 0.0]
    kr = [1.0, 1.0]
    krd = [0.3, 0.3] # 0.1
    kri = [0.1, 0.1] # 0.1
    krr = [0.001, 0.001] # 0.00005, sim = 0.0091, 0.001
    krrd = [0.0, 0.0]
    krri = [0.1, 0.1] # 0.1, sim = 0.976

    # physical params
    wing_radius = 200/1000 # change to 700 next round
    chord_length = 0.12
    mass = 75/1000
    cl = 0.5
    cd = 0.052
    J = np.array([1/100000,1/100000,1/1000000]) # moment of inertia
    
    monoco = monoco_att_ctrl.att_ctrl(kp, kd, ki, kvp, kvd, kvi, ka, kad, kai, kr, krd, kri, krr, krrd, krri)
    monoco.physical_params(wing_radius, chord_length, mass, cl, cd, J)        
    
    time_last = 0
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 
    
    # flatness option
    flatness_option = 0  # this is a must!
    amplitude = 1

    # Initialize references
    ref_pos_circle = np.array([0.0,0.0,0.0])
    ref_pos = np.array([0.0,0.0,0.0])
    ref_pos_z = 0.0
    ref_vel = np.array([0.0,0.0,0.0])
    ref_acc = np.array([0.0,0.0,0.0])
    ref_jerk = np.array([0.0,0.0,0.0])
    ref_snap = np.array([0.0,0.0,0.0])
    ref_msg = "havent computed yet"

    # Control Loop Commands
    cmd_att = np.array([0.0,0.0])
    cmd_bod_rates = np.array([0.0,0.0])
    final_cmd = np.array([[0.0,0.0,0.0,0.0]])

    # initial flatness terms
    ref_rates = np.array([0.0,0.0])
    ref_raterates = np.array([0.0,0.0])


    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
        cmd_att = np.array([cmd_att_startup])
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)

        try:
            #while time_end > time.time():
            while True:
                start = timeit.default_timer() 
                abs_time = time.time() - time_start

                # where hand control comes
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True

                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                lowest = 0.9
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

                # joystick saturation

                if conPad < 10:
                    conPad = 10
                if conPad > 65500:
                    conPad = 65500

                # takeoff sign
                if conPad < 2000:
                    enable = 0
                else:
                    enable = 1

                # require data from Mocap
                data = data_receiver_sender.get_data()

                # data unpack
                data_processor.data_unpack_filtered(data)

                # processed tpp data/feedback
                # rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_finite_diff()
                rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
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

                # time difference needed to calculate velocity
                dt = time.time() - time_last  #  time step/period
                time_last = time.time()

                # update positions etc.
                monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)

                # compute bem thrust
                monoco.compute_bem_wo_rps(body_pitch)


                # update references for PID loop
                if loop_counter % pid_loop == 0:
            
                    # hovering test
                    if stage == 'hover':
                        ref = traj_gen.hover_test(x_hover_offset,y_hover_offset)
                        
                        hovering_ff = np.array([0.0, 0.0, 0.0])
                        ref_pos = ref[0]
                        ref_pos_z = ref_pos[2]
                        ref_vel = hovering_ff
                        ref_acc = hovering_ff
                        ref_jerk = hovering_ff
                        ref_snap = hovering_ff
                        ref_msg = ref[1]


                        ## trajectory inputs
                        # if stage == 'trajectory on':
                        #     ref_derivatives = traj_gen.jerk_snap_circle(pva,num_pts,count,alt)
                        #     ref_pos = ref_derivatives[0]
                        #     ref_pos_z = ref_pos[2]
                        #     ref_vel = ref_derivatives[1]
                        #     ref_acc = ref_derivatives[2]
                        #     ref_jerk = ref_derivatives[3]
                        #     ref_snap = ref_derivatives[4]
                        #     ref_msg = ref_derivatives[5]

                        # update references for PID loop
                        monoco.update_ref_pos(ref_pos)

                        # ff references
                        monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap,ref_pos_z)

                        # control input (position + velocity)
                        monoco.p_control_input(1/(max_sample_rate/pid_loop))
                        monoco.v_control_input()

                        # flatness terms
                        ref_rates = monoco.ref_rates
                        ref_raterates = monoco.ref_raterates

                        final_cmd = monoco.CF_SAM_get_angles_and_thrust(flatness_option,amplitude)


                        """ # control input (arming test)
                        cmd_att_arm = np.array([0, 0, 0, conPad * 1]) # optitrack control [roll,  pitch ,  yawrate, thrust]
                        cmd_att = np.array([cmd_att_arm, cmd_att_arm, cmd_att_arm]) """
                        
                
                # control input (traj execution)
                cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)
                cmd_att = np.array([cmd_att_1])
                seq_args = swarm_exe(cmd_att)
                swarm.parallel(arm_throttle, args_dict=seq_args)

                count = count + 1
                if count % 10 == 0:
                    
                    #print(abs_time) # updating at 120 hz
                    print (ref_pos_1[1]) 
                    print('robot_position', robot[0], robot[1], robot[2])
                    print('robot ref z pos', ref_pos[2])
                    print('z pos_error', ref_pos[2]-robot[2])

                # rmse accumulation
                rmse_num = rmse_num + (ref_pos[2]-robot[2])**2
                
                # save data
                data_saver.add_item(abs_time,
                                    robot,ref_pos
                                    )

                if button0 == 1:
                    """ # for hovering test
                    ref_pos[2] = 0.15
                    # descend
                    att_robot_1.update(robot, dt, ref_pos, z_offset)
                    cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)
                    cmd_att = np.array([cmd_att_1])
                    seq_args = swarm_exe(cmd_att)
                    swarm.parallel(arm_throttle, args_dict=seq_args) """

                    # for traj 
                    cmd_att_cut = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
                    cmd_att = np.array([cmd_att_cut])
                    seq_args = swarm_exe(cmd_att)
                    swarm.parallel(init_throttle, args_dict=seq_args)
                    final_rmse = math.sqrt(rmse_num/count)
                    print('Emergency Stopped and rmse produced: ', final_rmse)
                    break

# save data
#path = '/home/emmanuel/AFC_Optitrack/robot_solo/'
#data_saver.save_data(path)

