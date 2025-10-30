import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import statistics
  
class all_trajectory(object):
    def __init__(self):
        pass

    def traj_data_compiler(self,file_path):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        #position = mat_data['data']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        att_raterate_error = mat_data['att_raterate_error']
        yawrate = mat_data['yawrate']

        start = int(np.size(time[0])*2/5)
        end = int(np.size(time[0])*3/5) #- int(np.size(time[0])

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        att_raterate_error_roll = [row[0] for row in att_raterate_error] # column vector
        att_raterate_error_pitch = [row[1] for row in att_raterate_error] 

        att_raterate_error_roll = np.array([att_raterate_error_roll])
        att_raterate_error_pitch = np.array([att_raterate_error_pitch])

        traj_time = time[0][start:end]
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        mf_px = ndimage.median_filter(px[0][start:end], size=300)
        mf_py = ndimage.median_filter(py[0][start:end], size=300)
        mf_pz = ndimage.median_filter(pz[0][start:end], size=300)
        att_raterate_error_roll = ndimage.median_filter(att_raterate_error_roll[0][start:end], size=700)
        att_raterate_error_pitch = ndimage.median_filter(att_raterate_error_pitch[0][start:end], size=700)
        
        yawrate = ndimage.median_filter(yawrate[0], size=200)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz

        xyz_error_norm = []
        rollpitch_raterate_error_norm = []
        roll_raterate_error_norm  = []
        pitch_raterate_error_norm = []  
        yawrate_ls = []     

        for i in range(len(mf_px)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))

            roll_raterate_error_squared = (att_raterate_error_roll[i]/10000) ** 2
            pitch_raterate_error_squared = (att_raterate_error_pitch[i]/10000) ** 2
            roll_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared))
            pitch_raterate_error_norm.append(math.sqrt(pitch_raterate_error_squared))
            rollpitch_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared + pitch_raterate_error_squared))
            
            yawrate_ls.append(yawrate[i])

            
        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            rollpitch_raterate_error_norm,
            roll_raterate_error_norm,
            pitch_raterate_error_norm,
            yawrate_ls
        )
    

    def att_data_compiler(self,file_path):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        #position = mat_data['data']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        att_raterate_error = mat_data['att_raterate_error']
        yawrate = mat_data['yawrate']

        start = 0
        end = int(np.size(time[0])) # int(np.size(time[0])

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        att_raterate_error_roll = [row[0] for row in att_raterate_error] # column vector
        att_raterate_error_pitch = [row[1] for row in att_raterate_error] 

        att_raterate_error_roll = np.array([att_raterate_error_roll])
        att_raterate_error_pitch = np.array([att_raterate_error_pitch])

        traj_time = time[0][start:end]
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        mf_px = ndimage.median_filter(px[0][start:end], size=700)
        mf_py = ndimage.median_filter(py[0][start:end], size=700)
        mf_pz = ndimage.median_filter(pz[0][start:end], size=700)
        att_raterate_error_roll = ndimage.median_filter(att_raterate_error_roll[0][start:end], size=1500)
        att_raterate_error_pitch = ndimage.median_filter(att_raterate_error_pitch[0][start:end], size=1500)
        
        yawrate = ndimage.median_filter(yawrate[0], size=300)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz

        xyz_error_norm = []
        rollpitch_raterate_error_norm = []
        roll_raterate_error_norm  = []
        pitch_raterate_error_norm = []  
        yawrate_ls = []  
        roll_raterate_error_sq = []
        pitch_raterate_error_sq = []   

        for i in range(len(mf_px)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))

            roll_raterate_error_squared = (att_raterate_error_roll[i]/10000) ** 2
            pitch_raterate_error_squared = (att_raterate_error_pitch[i]/10000) ** 2
            roll_raterate_error_sq.append(roll_raterate_error_squared)
            pitch_raterate_error_sq.append(pitch_raterate_error_squared)
            roll_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared))
            pitch_raterate_error_norm.append(math.sqrt(pitch_raterate_error_squared))
            rollpitch_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared + pitch_raterate_error_squared))
            
            yawrate_ls.append(yawrate[i])

        roll_raterate_rmse = math.sqrt(sum(roll_raterate_error_sq)/len(mf_px))
        pitch_raterate_rmse = math.sqrt(sum(pitch_raterate_error_sq)/len(mf_px))

        rmse_raterate_list = [roll_raterate_rmse,pitch_raterate_rmse]
        final_rmse_raterate = la.norm(rmse_raterate_list, 2)
            
        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            roll_raterate_error_norm,
            pitch_raterate_error_norm,
            rollpitch_raterate_error_norm,
            yawrate_ls,
            final_rmse_raterate
        )


    def normal_data_compiler(self,file_path):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        yawrate = mat_data['yawrate']
        motor_cmd = mat_data['motor_cmd']

        start = 0 #int(np.size(time[0])/10)
        end = int(np.size(time[0])) #- int(np.size(time[0])

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        traj_time = time[0][start:end]
        motor_cmd = ndimage.median_filter(motor_cmd[0][start:end], size=700)
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        mf_px = ndimage.median_filter(px[0][start:end], size=700)
        mf_py = ndimage.median_filter(py[0][start:end], size=700)
        mf_pz = ndimage.median_filter(pz[0][start:end], size=700)
        
        yawrate = ndimage.median_filter(yawrate[0], size=700)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz
        yawrate_ls = []

        xyz_error_norm = []      

        for i in range(len(mf_px)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))
            yawrate_ls.append(yawrate[i])

        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            yawrate_ls,
            motor_cmd
        )


    def data_whisker_plot(self,file_path):
        data = self.normal_data_compiler(file_path)
        mf_px = data[1]
        mf_py = data[2]
        mf_pz = data[3]
        px_r = data[4]
        py_r = data[5]
        pz_r = data[6]
        yawrate_ls = data[8]
        motor_cmd = data[9]
        traj_time = data[0]

        x_error = [] 
        y_error = []
        z_error = []
        x_error_squared = [] 
        y_error_squared = []
        z_error_squared = []
        x_error_norm = []
        y_error_norm = []
        z_error_norm = []
        y_error_norm = []
        z_error_norm = []

        for i in range(len(mf_px)):
            x_error_squared.append((px_r[i] - mf_px[i]) ** 2)
            y_error_squared.append((py_r[i] - mf_py[i]) ** 2)    
            z_error_squared.append((pz_r[i] - mf_pz[i]) ** 2)
            x_error_norm.append(math.sqrt(x_error_squared[i]))
            y_error_norm.append(math.sqrt(y_error_squared[i]))
            z_error_norm.append(math.sqrt(z_error_squared[i]))
            x_error.append((px_r[i] - mf_px[i]))
            y_error.append((py_r[i] - mf_py[i]))
            z_error.append((pz_r[i] - mf_pz[i]))

        final_rmse_x = math.sqrt(sum(x_error_squared)/len(mf_px))
        final_rmse_y = math.sqrt(sum(y_error_squared)/len(mf_px))
        final_rmse_z = math.sqrt(sum(z_error_squared)/len(mf_px))
        rmse_xyz_list = [final_rmse_x,final_rmse_y,final_rmse_z]
        final_rmse_xyz = la.norm(rmse_xyz_list, 2)

        return (
            x_error,
            y_error,
            z_error,
            x_error_squared,
            y_error_squared,
            z_error_squared,
            x_error_norm,
            y_error_norm,
            z_error_norm,
            rmse_xyz_list,
            final_rmse_xyz,
            traj_time,
            yawrate_ls,
            motor_cmd
        )

            




