import time

import Mocap
import DataSave
import Data_process
import numpy as np
import timeit
import math
import numpy.linalg as la
from pyrr import quaternion

if __name__ == '__main__':

    data_receiver_sender = Mocap.Udp()
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    ##data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'raw_data'
                                   )

    time_start = time.time()
    time_end = time_start + 1000
    last_time = 0

    UDP_IP = "192.168.65.221"
    UDP_PORT = 1234
    test_cmd_1 = float(3.423)
    test_cmd_2 = float(4.534)
    test_cmd_3 = float(5.645)
    test_cmd_4 = float(6.756)
    final_cmd = np.array([[test_cmd_1,test_cmd_2,test_cmd_3,test_cmd_4]])

    count = 0
    try:
        #while time_end > time.time():
        while True:  
            start = timeit.default_timer()  
            abs_time = time.time() - time_start

            # require data from Mocap
            data = data_receiver_sender.get_data()
            
            # data unpack
            # data_processor.data_unpack(data)
            data_processor.data_unpack_filtered(data)

            # cartesian position
            pos = data_processor.filted_data
            
            # processed tpp data/feedback
            state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
            
            # needa find pitch angle of the body during hover
            # data_processor.get_rotm_filtered()
            tpp_angle = data_processor.tpp 
            tpp_omega = data_processor.Omega
            tpp_omega_dot = data_processor.Omega_dot
            body_pitch = data_processor.body_pitch
            tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()
            body_roll = data_processor.body_angle_roll

            # testing sending information over udp to wj esp32
            #print ("sending info: ", final_cmd)
            #data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)

            count = count + 1

            if count % 5 == 0:
                #print('-----', 'time: ', abs_time, '-----')
                #print(raw_data)

                # save data
                
                #data_saver.add_item(abs_time,
                #                    raw_data
                #                    )
                
                t_diff = abs_time - last_time
                last_time = abs_time

                r11 = data_processor.R11
                r12 = data_processor.R12
                r13 = data_processor.R13
                r21 = data_processor.R21
                r22 = data_processor.R22
                r23 = data_processor.R23
                r31 = data_processor.R31
                r32 = data_processor.R32
                r33 = data_processor.R33

                
                #print("sampling period and freq: ", t_diff, 1/t_diff) 
                print("tpp angles in degrees:", round((tpp_angle[0]*(180/np.pi)),3),round((tpp_angle[1]*(180/np.pi)),3)) # rpy
                #print("tpp bodypitch:", body_pitch)
                #print("tpp bodyrates:", tpp_omega) # rpy
                #print("tpp bodyraterates:", tpp_omega_dot) # rpy
                print("position: ", pos[0:3])
                #print("tpp quaternion: ", tpp_quat)

                
                #print("r11, r12, r13: ", r11/n, r12/n, r13/n)
                #print("r21, r22, r23: ", r21/n, r22/n, r23/n)

                # r11 abt y, r22 abt x
                
                # roll_vector = np.array([r11, r21, r31]) # SO3 x axis vector
                # pitch_vector = np.array([r12, r22, r32]) # SO3 y axis vector
                
                # z_vector = np.array([0, 0, 1])
                # x_vector = np.array([0, 1, 0])
                # roll = np.dot(roll_vector,z_vector) # 1 x 1
                # pitch = np.dot(pitch_vector,z_vector)

                tpp_quat_y = np.array([tpp_quat[2][0], tpp_quat[2][1], tpp_quat[2][2], 1.0]) # xy0w
                
                #tpp_quat = np.array([tpp_quat[0], tpp_quat[1], tpp_quat[2], 1.0]) # xyzw
                tpp_quat_inverse = quaternion.inverse(tpp_quat_y)
                ez = np.array([0, 0, 1])
                disk_vector = quaternion.apply_to_vector(tpp_quat_y, ez) # flattened array

                
                #print(disk_vector)
                yaw = math.atan2(r11,r21)
                # denominator is 1 as its a unit vector (quaternion mag is 1)
                # bod_pitch = math.acos(pitch) 
                # bod_roll = math.acos(roll) 
                
                # pitch_rad = np.pi/2 - bod_pitch
                yaw_deg = round(yaw*(180/np.pi),2)
                body_roll = round(body_roll*(180/np.pi),2)
                print ("yaw_deg: ", yaw_deg)
                print ("bod_roll_deg, pitch deg: ", body_roll, body_pitch)

                # ## tpp roll
                # tpp_roll = np.pi/2 - bod_roll
                # tpp_pitch = np.pi/2 - bod_roll
                # if abs(yaw_deg) > 90.0:
                #     tpp_roll = -1*tpp_roll
                #print("tpp_roll: ", tpp_roll)

                ## tpp pitch
                # if yaw_deg > 0:
                #     tpp_pitch = -1*tpp_pitch
                #print("tpp_pitch: ", tpp_pitch)
                                
                #print(round(roll*(180/np.pi),3), round(roll*(180/np.pi),3))
                #print("cos yaw: ", math.cos(yaw))
                
                #print("connected....")
            
    
                #time.sleep(0.05) # impt to pause and see information
            stop = timeit.default_timer()
            #print('Program Runtime: ', stop - start)  

    except KeyboardInterrupt:
        print('Keyboard interrupt')
        
# save data
#path = '/home/emmanuel/AFC_Optitrack/linux_data/'
#data_saver.save_data(path)

