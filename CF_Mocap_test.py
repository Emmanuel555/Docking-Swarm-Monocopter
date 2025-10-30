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
                                   'Monocopter_XYZ_raw','Monocopter_XYZ','tpp_roll','tpp_pitch','body_yaw_deg')    
      
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
            #data_processor.data_unpack(data)
            #data_processor.data_filtered()
            data_processor.data_unpack_filtered(data)

            # cartesian position
            pos = data_processor.filted_data
            #pos = data_processor.raw_data
            # raw cartesian position
            pos_raw = data_processor.raw_data
            
            # processed tpp data/feedback
            state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
            
            # needa find pitch angle of the body during hover
            # data_processor.get_rotm_filtered()
            tpp_angle = data_processor.tpp 
            tpp_omega = data_processor.Omega
            tpp_omega_dot = data_processor.Omega_dot
            body_pitch = data_processor.body_pitch
            tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()

            count = count + 1

            t_diff = abs_time - last_time
            last_time = abs_time

            tpp_roll = round((tpp_angle[0]*(180/np.pi)),3)
            tpp_pitch = round((tpp_angle[1]*(180/np.pi)),3)

            yaw = data_processor.yaw
            yaw_deg = round(yaw*(180/np.pi),2)

            if count % 5 == 0:
               
                #print("sampling period and freq: ", t_diff, 1/t_diff) 
                print("tpp angles in degrees:", tpp_roll, tpp_pitch) # rpy
                #print("tpp bodypitch:", body_pitch)
                #print("tpp bodyrates:", tpp_omega) # rpy
                #print("tpp bodyraterates:", tpp_omega_dot) # rpy
                print("position: ", pos[0:3])
                #print("tpp quaternion: ", tpp_quat)
 
                print ("yaw_deg: ", yaw_deg)

                #time.sleep(0.05) # impt to pause and see information
            stop = timeit.default_timer()
            #print('Program Runtime: ', stop - start)  

            data_saver.add_item(abs_time,
                                pos_raw[0:3],pos[0:3],tpp_roll,tpp_pitch,yaw_deg)
            #data_saver.add_item(abs_time,
            #                    data)

    except KeyboardInterrupt:
        data_saver.add_item(abs_time,
                            pos_raw[0:3],pos[0:3],tpp_roll,tpp_pitch,yaw_deg)
        print('Keyboard interrupt')

        

# save data
path = '/home/emmanuel/Monocopter-OCP/cf_robot_solo/'
data_saver.save_data(path)

