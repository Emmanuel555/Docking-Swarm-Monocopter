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

    #rate_loop = 1
    #att_loop = 1
    #pid_loop = 5 # 5 is the minimum

    # rate_loop = raterate_loop * 3 # 90 hz, nt good 
    # att_loop = rate_loop * 1 # 120  nt good
    # pid_loop = rate_loop * 1 # 120, nt good

    #att_loop = rate_loop * 1.5 # 3, 120 hz not good
    #pid_loop = rate_loop * 1.5 # 3, 120 hz not good 

    #att_loop = rate_loop * 3 # 6, 60 hz second best 
    #pid_loop = rate_loop * 3 # 6, 60 hz second best

    #att_loop = rate_loop * 6 # 30, 12 hz not good
    #pid_loop = rate_loop * 6 # 30, 12 hz not good

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

    # Monocopter UDP IP & Port
    UDP_IP = "192.168.65.221"
    UDP_PORT = 1234

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


    try:
        #while time_end > time.time():
        while True:
            start = timeit.default_timer() 
            abs_time = time.time() - time_start
            
            #lowest = 0.9
            #highest = -1
            #range_js = -(highest - lowest)
            #range_motor = 65535
            #rate = range_motor / range_js
        
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

            # reference position
            # ref = traj_gen.simple_rectangle(x_offset, y_offset, abs_time)
        
            #ref_pos = traj_gen.simple_circle(0, 0.25, count, 5)
            #ref_pos = traj_gen.elevated_circle(0, 0.6, count)
            
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

                

            #ref_pos_1 = traj_gen.helix(0, 0.4, count, 5)
            #ref_pos = ref_pos_1[0]

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

                #final_cmd = monoco.get_angles_and_thrust(flatness_option,amplitude)

                # send to monocopter via INDI
                #data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)


            if loop_counter % att_loop == 0:

                # get angle
                cmd_att = monoco.get_angle(1/(max_sample_rate/att_loop))

            
            if loop_counter % rate_loop == 0:

                # bod rates
                monoco.get_body_rate(cmd_att,flatness_option,1/(max_sample_rate/rate_loop),amplitude)
            

            #if loop_counter % (raterate_loop*3) == 0:
                # final control input (INDI loop)
                #final_cmd = monoco.get_angles_and_thrust(flatness_option,amplitude)

                # send to monocopter via INDI
                #data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)

            # collective thrust
            z_controls = monoco.collective_thrust(kpz,kdz,kiz)
            z_control_signal = z_controls[0]
            des_thrust = z_controls[1]

            # precession rate
            precession_yaw_rate = monoco.precession_rate()

            # final control input (INDI loop)
            final_cmd = monoco.get_angles_and_thrust(flatness_option,amplitude)

            # send to monocopter via INDI
            data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)

            loop_counter = loop_counter + 1


            # normal counter to visibly see the updates on terminal
            count = count + 1
            if count % 10 == 0:
                
                #print(abs_time) # updating at 120 hz
                print(ref_msg) 
                #print('shapes: ', np.shape(final_cmd))
                print('cmd info sent: ', final_cmd)
                #print('tpp_position', linear_state_vector[0], linear_state_vector[1], linear_state_vector[2])
                #print('tpp_angle_from_rotational', rotational_state_vector[0][0], rotational_state_vector[0][1], rotational_state_vector[0][2])
                #print('tpp_angle', tpp_angle)
                print('stage', stage)
                #print('tpp_omega', type(tpp_omega),type(rotational_state_vector[1]))
                #print('ref robot_position', ref_pos[0], ref_pos[1], ref_pos[2])
                #print('pos_error', ref_pos[0]-linear_state_vector[0], ref_pos[1]-linear_state_vector[1], ref_pos[2]-linear_state_vector[2])


            # start experiment
            #if count > (20 * max_sample_rate): # 5 seconds
            status = stage


            # rmse accumulation
            rmse_num_x = rmse_num_x + (ref_pos[0]-x_offset-linear_state_vector[0])**2
            rmse_num_y = rmse_num_y + (ref_pos[1]-y_offset-linear_state_vector[1])**2
            rmse_num_z = rmse_num_z + (ref_pos[2]-z_offset-linear_state_vector[2])**2
            rmse_num = [rmse_num_x, rmse_num_y, rmse_num_z]

            # save data
            data_saver.add_item(abs_time,
                                linear_state_vector[0:3],ref_pos,rmse_num,0,ref_msg,status,final_cmd,tpp_angle,tpp_omega,tpp_omega_dot,linear_state_vector[3:6],z_control_signal,des_thrust,ref_rates,ref_raterates,precession_yaw_rate[0],precession_yaw_rate[1])
            
            stop = timeit.default_timer()
            #print('Program Runtime: ', stop - start)  
                                
            
    except KeyboardInterrupt:
            
            # final rmse calculation
            status = "Emergency stop"
            ref_msg = "traj ended..."
            final_rmse_x = math.sqrt(rmse_num_x/count)
            final_rmse_y = math.sqrt(rmse_num_y/count)
            final_rmse_z = math.sqrt(rmse_num_z/count)
            final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
            rmse_num = [final_rmse_x, final_rmse_y, final_rmse_z]
            
            # save data
            data_saver.add_item(abs_time,
                                linear_state_vector[0:3],ref_pos,rmse_num,final_rmse,ref_msg,status,final_cmd,tpp_angle,tpp_omega,tpp_omega_dot,linear_state_vector[3:6],z_control_signal,des_thrust,ref_rates,ref_raterates,precession_yaw_rate[0],precession_yaw_rate[1])

            print('Emergency Stopped and final rmse produced: ', final_rmse)
            

# save data
#path = '/home/emmanuel/Monocopter-OCP/robot_solo/2ptlinehover_test_0.0,0.0_hgt_1.5'
#path = '/home/emmanuel/Monocopter-OCP/robot_solo/circle_test_1x_0.5_hgt_1.5'
#path = '/home/emmanuel/Monocopter-OCP/robot_solo/circle_INDI_df_x1_0.5_hgt_1.5'
#path = '/home/emmanuel/Monocopter-OCP/robot_solo/att_tracking_not_filtered_collective0.0_clean_gains_0.1,0.0,0.01_attraterate360_0.1,0.0,0.01_attrate360_0.1,0.0,0.01_att360_0.1_vel36_1.0,0.0,0.1_pid36_tpp_sim_-90'
#data_saver.save_data(path)
