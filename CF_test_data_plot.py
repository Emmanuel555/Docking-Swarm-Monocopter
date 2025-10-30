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
position_raw = mat_data['Monocopter_XYZ_raw']
tpp_roll = mat_data['tpp_roll']
tpp_pitch = mat_data['tpp_pitch']
body_yaw = mat_data['body_yaw_deg']
tpp_omega = mat_data['tpp_omega']
tpp_omega_dot = mat_data['tpp_omega_dot']
body_angle_roll = mat_data['body_angle_roll']

px = [row[0] for row in position] # column vector
py = [row[1] for row in position]
pz = [row[2] for row in position]

px = np.array([px])
py = np.array([py])
pz = np.array([pz])

px_r = [row[0] for row in position_raw] # column vector
py_r = [row[1] for row in position_raw]
pz_r = [row[2] for row in position_raw]

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

## generate the plot
#plt.figure(figsize=(10, 5))
fig, ((ax1,ax2), (ax3,ax4)) = plt.subplots(2, 2, figsize=(40, 10))
#print(cmd_x[0])



sampling_freq = 250
start = int(np.size(time[0])/3)
end = np.size(time[0]) - int(np.size(time[0])/3)
v_start = 0
v_end = np.size(time[0]) - int(np.size(time[0])/3)
v_x = []
v_y = []


print('time taken: ', (end - start)/sampling_freq)
median_filtered_px = ndimage.median_filter(px[0][start:end], size=100)
median_filtered_py = ndimage.median_filter(py[0][start:end], size=100)
median_filtered_pz = ndimage.median_filter(pz[0][start:end], size=100)


for i in range(len(px[0][v_start:v_end])):
    vx = (px[0][i+1] - px[0][i])/(time[0][i+1] - time[0][i])
    vy = (py[0][i+1] - py[0][i])/(time[0][i+1] - time[0][i])
    v_x.append(vx)
    v_y.append(vy)


ref_x = 1.0
ref_y = 1.0
ref_z = 1.0

x_error = [] 
y_error = []
z_error = []
x_error_squared = [] 
y_error_squared = []
z_error_squared = []

for i in range(len(median_filtered_px)):
    x_error_squared.append((ref_x - median_filtered_px[i]) ** 2)
    y_error_squared.append((ref_y - median_filtered_py[i]) ** 2)    
    z_error_squared.append((ref_z - median_filtered_pz[i]) ** 2)
    x_error.append((ref_x - median_filtered_px[i]))
    y_error.append((ref_y - median_filtered_py[i]))
    z_error.append((ref_z - median_filtered_pz[i]))
    
final_rmse_x = math.sqrt(sum(x_error_squared)/(end-start))
final_rmse_y = math.sqrt(sum(y_error_squared)/(end-start))
final_rmse_z = math.sqrt(sum(z_error_squared)/(end-start))
final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)

x_med = statistics.median(x_error)
y_med = statistics.median(y_error)
z_med = statistics.median(z_error)  
print('error med: ', x_med, y_med, z_med)
print('Final RMSE: ', final_rmse_y)

ax1.plot(time[0][start:end], x_error, label='x err', color='blue',linewidth=2)
ax1.plot(time[0][start:end], y_error, label='y err', color='red',linewidth=2)
ax1.plot(time[0][start:end], z_error, label='z err', color='green',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_px, label='x', color='blue',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_py, label='y', color='red',linewidth=2)
#ax1.plot(time[0][start:end], median_filtered_pz, label='z', color='green',linewidth=2)
# ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
ax1.legend()
ax1.set_title('Position vs time', fontsize=20)
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Position(m)')

plt.subplots_adjust(hspace=0.34, wspace=0.2)


v_x = ndimage.median_filter(v_x, size=30)
v_y = ndimage.median_filter(v_y, size=30)

ax2.plot(time[0][v_start:v_end], v_x, label='vx', color='blue',linewidth=2)
ax2.plot(time[0][v_start:v_end], v_y, label='vy', color='red',linewidth=2)
#ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
# ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
ax2.legend()
ax2.set_title('Velocity', fontsize=20)
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Velocity (m/s)')


# ax2.plot(time[0], tpp_roll[0], label='tpp_roll_deg', color='blue')
# ax2.plot(time[0], tpp_pitch[0], label='tpp_pitch_deg', color='red')
# ax2.legend()
# ax2.set_title('TPP angles vs time', fontsize=20)
# ax2.set_xlabel('Time(s)')
# ax2.set_ylabel('Angle(deg)')

ax3.plot(time[0], np.round((tpp_roll_rate[0]*(180/np.pi)),3), label='tpp_roll_rate', color='blue')
ax3.plot(time[0], np.round((tpp_pitch_rate[0]*(180/np.pi)),3), label='tpp_pitch_rate', color='red')
ax3.legend()
ax3.set_title('TPP angle/s(deg) vs time', fontsize=20)
ax3.set_xlabel('Time(s)')
ax3.set_ylabel('Angle/s(deg)')

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

ax4.plot(time[0], np.round((body_angle_roll[0]*(180/np.pi)),3), label='body_roll_deg', color='black')
ax4.legend()
ax4.set_title('Body_roll(deg) vs time', fontsize=20)
ax4.set_xlabel('Time(s)')
ax4.set_ylabel('Angle(deg)')



# Show the figure
plt.show()

