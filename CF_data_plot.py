import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la
from Filter import IIR2Filter

file_path = '/home/emmanuel/Monocopter-OCP/cf_robot_solo/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
position = mat_data['Monocopter_XYZ']
position_raw = mat_data['Monocopter_XYZ_raw']
p_ref = mat_data['ref_position']
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

ref_x = [row[0] for row in p_ref] # column vector
ref_y = [row[1] for row in p_ref]
ref_z = [row[2] for row in p_ref]

ref_x = np.array([ref_x])
ref_y = np.array([ref_y])
ref_z = np.array([ref_z])

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

ax1.plot(time[0], px[0], label='x', color='blue',linewidth=2)
ax1.plot(time[0], py[0], label='y', color='red',linewidth=2)
ax1.plot(time[0], pz[0], label='z', color='green',linewidth=2)
ax1.plot(time[0], px_r[0], label='x_r', color='blue',linewidth=2,linestyle='dashed')
ax1.plot(time[0], py_r[0], label='y_r', color='red',linewidth=2,linestyle='dashed')
ax1.plot(time[0], pz_r[0], label='z_r', color='green',linewidth=2,linestyle='dashed')
ax1.legend()
ax1.set_title('Position vs time', fontsize=20)
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Position(m)')

plt.subplots_adjust(hspace=0.34, wspace=0.2)

ax2.plot(time[0], ref_x[0], label='x_input', color='blue')
#ax2.plot(time[0], tpp_pitch[0], label='tpp_pitch_deg', color='red')
ax2.legend()
ax2.set_title('P control input', fontsize=20)
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Magnitude')


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

