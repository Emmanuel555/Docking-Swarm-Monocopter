import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la
from Filter import IIR2Filter
import statistics
  

file_path = '/home/emmanuel/Monocopter-OCP/cf_data_selected/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
position = mat_data['Monocopter_XYZ']
ref_position = mat_data['ref_position']
tpp_roll = mat_data['tpp_roll']
tpp_pitch = mat_data['tpp_pitch']
body_yaw = mat_data['body_yaw_deg']
tpp_omega = mat_data['tpp_omega']
tpp_omega_dot = mat_data['tpp_omega_dot']
body_angle_roll = mat_data['body_angle_roll']
att_error = mat_data['att_error']
yawrate = mat_data['yawrate']
rmse = mat_data['rmse_num_xyz']

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


tpp_roll_rate = [row[0] for row in tpp_omega] # column vector
tpp_pitch_rate = [row[1] for row in tpp_omega]

tpp_roll_raterate = [row[0] for row in tpp_omega_dot] # column vector
tpp_pitch_raterate = [row[1] for row in tpp_omega_dot]

tpp_roll_rate = np.array([tpp_roll_rate])
tpp_pitch_rate = np.array([tpp_pitch_rate])

tpp_roll_raterate = np.array([tpp_roll_raterate])
tpp_pitch_raterate = np.array([tpp_pitch_raterate])

x_e = [row[0] for row in rmse] # column vector
y_e = [row[1] for row in rmse]
z_e = [row[2] for row in rmse]

x_e = np.array([x_e])
y_e = np.array([y_e])
z_e = np.array([z_e])


sampling_freq = 250
start = 0 #int(np.size(time[0])/4)
end = int(np.size(time[0])) #- int(np.size(time[0])/6)
v_x = []
v_y = []


print('time taken: ', (end - start)/sampling_freq)
traj_time = time[0][start:end]
#mf_x_e = ndimage.median_filter(x_e[0][start:end], size=200)
#mf_y_e = ndimage.median_filter(y_e[0][start:end], size=200)
#mf_z_e = ndimage.median_filter(z_e[0][start:end], size=200)
mf_px = ndimage.median_filter(px[0][start:end], size=300)
mf_py = ndimage.median_filter(py[0][start:end], size=300)
mf_pz = ndimage.median_filter(pz[0][start:end], size=300)


print('len: ', len(mf_px)) # must use x_e[0] to get the length of the array


for i in range(len(mf_px)-1):
    vx = (mf_px[i+1] - mf_px[i])/(traj_time[i+1] - traj_time[i])
    vy = (mf_py[i+1] - mf_py[i])/(traj_time[i+1] - traj_time[i])
    v_x.append(vx)
    v_y.append(vy)


x_error = [] 
y_error = []
z_error = []
x_error_squared = [] 
y_error_squared = []
z_error_squared = []


for i in range(len(mf_px)):
    x_error_squared.append((px_r[0][i] - mf_px[i]) ** 2)
    y_error_squared.append((py_r[0][i] - mf_py[i]) ** 2)    
    z_error_squared.append((pz_r[0][i] - mf_pz[i]) ** 2)
    x_error.append((px_r[0][i] - mf_px[i]))
    y_error.append((py_r[0][i] - mf_py[i]))
    z_error.append((pz_r[0][i] - mf_pz[i]))

# for i in range(len(mf_x_e)):
#     x_error_squared.append((mf_x_e[i]) ** 2)
#     y_error_squared.append((mf_y_e[i]) ** 2)    
#     z_error_squared.append((mf_z_e[i]) ** 2)
#     # x_error.append((ref_x - median_filtered_px[i]))
#     # y_error.append((ref_y - median_filtered_py[i]))
#     # z_error.append((ref_z - median_filtered_pz[i]))
    
print('x error len: ', len(mf_px))
final_rmse_x = math.sqrt(sum(x_error_squared)/(end-start))
final_rmse_y = math.sqrt(sum(y_error_squared)/(end-start))
final_rmse_z = math.sqrt(sum(z_error_squared)/(end-start))
final_rmse = la.norm([final_rmse_x, final_rmse_y,final_rmse_z], 2)


x_med = statistics.median(x_error)
y_med = statistics.median(y_error)
z_med = statistics.median(z_error)  
med_err = la.norm([x_med, y_med, z_med], 2)
print('error med: ', x_med, y_med, z_med)
print('error med norm: ', med_err)
print('Final RMSE: ', final_rmse)
print('RMSE XYZ: ', final_rmse_x, final_rmse_y, final_rmse_z)


## generate the plot
#plt.figure(figsize=(10, 5))
fig, ((ax1,ax2), (ax3,ax4)) = plt.subplots(2, 2, figsize=(40, 10))
#print(cmd_x[0])


ax1.plot(traj_time, x_error, label='x err mf', color='blue',linewidth=2)
ax1.plot(traj_time, y_error, label='y err mf', color='red',linewidth=2)
ax1.plot(traj_time, z_error, label='z err mf', color='green',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_px, label='x', color='blue',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_py, label='y', color='red',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_pz, label='z', color='green',linewidth=2)
# ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
ax1.legend()
ax1.set_title('Position error vs time', fontsize=20)
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Position error(m)')
#ax1.set_title('Position vs time', fontsize=20)
#ax1.set_xlabel('Time(s)')
#ax1.set_ylabel('Position(m)')

plt.subplots_adjust(hspace=0.34, wspace=0.2)


ax2.plot(mf_px, mf_py, label='circle path', color='blue',linewidth=2)
ax2.plot(px_r[0][start:end], py_r[0][start:end], label='ref', color='red',linewidth=2)
#ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
ax2.legend()
ax2.set_title('X vs Y', fontsize=20)
ax2.set_xlabel('X(m)')
ax2.set_ylabel('Y(m)')


# ax2.plot(time[0], tpp_roll[0], label='tpp_roll_deg', color='blue')
# ax2.plot(time[0], tpp_pitch[0], label='tpp_pitch_deg', color='red')
# ax2.legend()
# ax2.set_title('TPP angles vs time', fontsize=20)
# ax2.set_xlabel('Time(s)')
# ax2.set_ylabel('Angle(deg)')

# ax3.plot(time[0], np.round((tpp_roll_rate[0]*(180/np.pi)),3), label='tpp_roll_rate', color='blue')
# ax3.plot(time[0], np.round((tpp_pitch_rate[0]*(180/np.pi)),3), label='tpp_pitch_rate', color='red')
# ax3.legend()
# ax3.set_title('TPP angle/s(deg) vs time', fontsize=20)
# ax3.set_xlabel('Time(s)')
# ax3.set_ylabel('Angle/s(deg)')



att_error = ndimage.median_filter(att_error[0], size=10)
ax3.plot(traj_time, np.round((att_error[start:end]*(180/np.pi)),2), label='att_error_norm', color='red')
ax3.legend()
ax3.set_title('Att_error_norm vs time', fontsize=20)
ax3.set_xlabel('Time(s)')
ax3.set_ylabel('Deg')



# ax3.plot(traj_time[0:-1], v_x, label='vx', color='blue',linewidth=2)
# ax3.plot(traj_time[0:-1], v_y, label='vy', color='red',linewidth=2)
# #ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
# # ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
# # ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
# # ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
# ax3.legend()
# ax3.set_title('Velocity', fontsize=20)
# ax3.set_xlabel('Time(s)')
# ax3.set_ylabel('Velocity (m/s)')


# ax4.plot(time[0], np.round((tpp_roll_raterate[0]*(180/np.pi)),3), label='tpp_roll_raterate', color='blue')
# ax4.plot(time[0], np.round((tpp_pitch_raterate[0]*(180/np.pi)),3), label='tpp_pitch_raterate', color='red')
# ax4.legend()
# ax4.set_title('TPP angle/s^2(deg) vs time', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Angle/s^2(deg)')

# ax4.plot(time[0], body_yaw[0], label='body_yaw_deg', color='black')
# ax4.legend()
# ax4.set_title('Body_yaw(deg) vs time', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Angle(deg)')

# ax4.plot(time[0], np.round((body_angle_roll[0]*(180/np.pi)),3), label='body_roll_deg', color='black')
# ax4.legend()
# ax4.set_title('Body_roll(deg) vs time', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Angle(deg)')

yawrate = ndimage.median_filter(yawrate[0], size=200)
ax4.plot(traj_time, np.round((yawrate[start:end]/(2*math.pi)),2), label='rotation rate in hz', color='red')
ax4.legend()
ax4.set_title('Yawrate(Hz) vs time', fontsize=20)
ax4.set_xlabel('Time(s)')
ax4.set_ylabel('Hz')



# Show the figure
plt.show()

