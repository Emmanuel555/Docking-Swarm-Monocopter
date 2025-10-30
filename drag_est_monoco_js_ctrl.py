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


if __name__ == '__main__':

    data_receiver_sender = Mocap.Udp()
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'Monocopter_XYZ','ref_position','rmse_num_xyz','final_rmse')
                                   
    logging.basicConfig(level=logging.ERROR)
    
    
    # reference offset for z
    z_offset = 0.0

    # rmse terms
    rmse_num_x = 0
    final_rmse_x = 0
    rmse_num_y = 0
    final_rmse_y = 0
    rmse_num_z = 0
    final_rmse_z = 0

    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()

    # traj generator for min snap circle, ####### pre computed points
    # pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle(0, 0.5, 1)
    pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(0, 2.0, 50, 5)  # (0, 1.2, 30) - 3 m/s, (0 , 1.4, 50) - 5m/s

    # collective z
    kpz = 9.6
    kdz = 2.5
    kiz = 64.0
    
    # cyclic xyz
    kp = [0.025,0.025,kpz]
    kd = [2,2,kdz]
    ki = [0,0,kiz]

    # cyclic xy
    ka = [1.0, 1.0]
    kad = [0.1, 0.1]
    kr = [0.1, 0.1]
    krd = [0.1, 0.1]
    krr = [1.0, 1.0]

    # physical params
    wing_radius = 320/1000
    chord_length = 0.12
    mass = 75/1000
    cl = 0.5
    cd = 0.052
    
    monoco = monoco_att_ctrl.att_ctrl(kp, kd, ki, ka, kad, kr, krd, krr)
    monoco.physical_params(wing_radius, chord_length, mass, cl, cd)        
    
    time_last = 0
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 

    # flatness option
    flatness_option = 0

    # Monocopter UDP IP & Port
    UDP_IP = "192.168.65.221"
    UDP_PORT = 1234


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
            rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul()
            linear_state_vector = data_processor.pos_vel_acc_filtered()
            body_pitch = data_processor.body_pitch
            # tpp_angle = data_processor.tpp
            # tpp_omega = data_processor.Omega
            # tpp_omega_dot = data_processor.Omega_dot
            tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()


            # time difference needed to calculate velocity
            dt = time.time() - time_last  #  time step/period
            time_last = time.time()


            # reference position
            #ref_pos = traj_gen.simple_rectangle(0, abs_time)
            #ref_pos = traj_gen.simple_circle(0, 0.25, count, 5)
            #ref_pos = traj_gen.elevated_circle(0, 0.6, count)
            
            # hovering test
            ref = traj_gen.hover_test(0)
            hovering_ff = np.array([0.0, 0.0, 0.0])
            ref_pos = ref[0]
            ref_vel = hovering_ff
            ref_acc = hovering_ff
            ref_jerk = hovering_ff
            ref_snap = hovering_ff
            ref_msg = ref[1]

            #ref_pos_1 = traj_gen.helix(0, 0.4, count, 5)
            #ref_pos = ref_pos_1[0]

            # ref_derivatives = traj_gen.jerk_snap_circle(pva,num_pts,count,0.25)
            # ref_pos = ref_derivatives[0]
            # ref_vel = ref_derivatives[1]
            # ref_acc = ref_derivatives[2]
            # ref_jerk = ref_derivatives[3]
            # ref_snap = ref_derivatives[4]
            # ref_msg = ref_derivatives[5]


            # update positions etc.
            monoco.update(linear_state_vector, rotational_state_vector, tpp_quat, dt, ref_pos, z_offset)
            
            # ff references
            monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap)

            # compute bem thrust
            monoco.compute_bem_wo_rps(body_pitch)
            
            # control input (traj execution)
            final_cmd = monoco.get_angles_and_thrust(flatness_option)

            # send to monocopter via INDI
            data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)

            # counter
            count = count + 1
            if count % 10 == 0:
                
                #print(abs_time) # updating at 120 hz
                print(ref_msg) 
                #print('shapes: ', np.shape(final_cmd))
                print('cmd info sent: ', final_cmd)
                print('tpp_position', linear_state_vector[0], linear_state_vector[1], linear_state_vector[2])
                print('ref robot_position', ref_pos[0], ref_pos[1], ref_pos[2])
                print('pos_error', ref_pos[0]-linear_state_vector[0], ref_pos[1]-linear_state_vector[1], ref_pos[2]-linear_state_vector[2])

            # rmse accumulation
            rmse_num_x = rmse_num_x + (ref_pos[0]-linear_state_vector[0])**2
            rmse_num_y = rmse_num_y + (ref_pos[1]-linear_state_vector[1])**2
            rmse_num_z = rmse_num_z + (ref_pos[2]-linear_state_vector[2])**2
            rmse_num = [rmse_num_x, rmse_num_y, rmse_num_z]
            
            # save data
            data_saver.add_item(abs_time,
                                linear_state_vector[0:3],ref_pos,rmse_num,0)
            
            stop = timeit.default_timer()
            #print('Program Runtime: ', stop - start)  
                                
            
    except KeyboardInterrupt:
            
            # final rmse calculation
            final_rmse_x = math.sqrt(rmse_num_x/count)
            final_rmse_y = math.sqrt(rmse_num_y/count)
            final_rmse_z = math.sqrt(rmse_num_z/count)
            final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
            rmse_num = [final_rmse_x, final_rmse_y, final_rmse_z]
            
            # save data
            data_saver.add_item(abs_time,
                                linear_state_vector[0:3],ref_pos,rmse_num,final_rmse)

            print('Emergency Stopped and final rmse produced: ', final_rmse)
            

# save data
#path = '/home/emmanuel/Monocopter-OCP/robot-solo/'
#data_saver.save_data(path)
