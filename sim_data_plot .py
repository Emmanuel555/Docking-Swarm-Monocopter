import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la
from Filter import IIR2Filter

file_path = '/home/emmanuel/Monocopter-OCP/data_selected/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
position = mat_data['Monocopter_XYZ']
velocity = mat_data['velocity']
z_control = mat_data['z_control']
des_thrust = mat_data['des_thrust']
precession = mat_data['yawrate']
print('Precession: ', precession[0][200])
px = [row[0] for row in position] # column vector
py = [row[1] for row in position]
pz = [row[2] for row in position]

vx = [row[0] for row in velocity] # column vector
vy = [row[1] for row in velocity]
vz = [row[2] for row in velocity]

cmd = mat_data['cmd']
tpp = mat_data['tpp_angle']
tpp_omega = mat_data['tpp_omega']
tpp_omega_dot = mat_data['tpp_omega_dot']
ref_rates = mat_data['ref_rates']
ref_raterates = mat_data['ref_raterates']

cmds = [row for row in cmd] # column vector
cmd_x = [row[0][0] for row in cmds] # column vector
cmd_y = [row[0][1] for row in cmds]
collective_z = [row[0][2] for row in cmds]

tpp_roll = [row[0] for row in tpp] # column vector
tpp_pitch = [row[1] for row in tpp]

tpp_roll_rate = [row[0] for row in tpp_omega] # column vector
tpp_pitch_rate = [row[1] for row in tpp_omega]

tpp_roll_raterate = [row[0] for row in tpp_omega_dot] # column vector
tpp_pitch_raterate = [row[1] for row in tpp_omega_dot]

ref_rates_abt_x = [row[0] for row in ref_rates] # column vector
ref_rates_abt_y = [row[1] for row in ref_rates]


ref_raterates_abt_x = [row[0] for row in ref_raterates] # column vector
ref_raterates_abt_y = [row[1] for row in ref_raterates]

cmd_x = np.array([cmd_x])
cmd_y = np.array([cmd_y])
collective_z = np.array([collective_z])

#z_control = np.array([z_control])
#print(np.size(z_control))

tpp_roll = np.array([tpp_roll])
tpp_pitch = np.array([tpp_pitch])



tpp_roll_rate = np.array([tpp_roll_rate])
tpp_pitch_rate = np.array([tpp_pitch_rate])

tpp_roll_raterate = np.array([tpp_roll_raterate])
tpp_pitch_raterate = np.array([tpp_pitch_raterate])

ref_rates_abt_x = np.array([ref_rates_abt_x])
ref_rates_abt_y = np.array([ref_rates_abt_y])
ref_raterates_abt_x = np.array([ref_raterates_abt_x])
ref_raterates_abt_y = np.array([ref_raterates_abt_y])

px = np.array([px])
py = np.array([py])
pz = np.array([pz])

vx = np.array([vx])
vy = np.array([vy])
vz = np.array([vz])

sample_rate = 360
order = 10
cutoff = 1
ftype = 'lowpass'
design = 'butter'
filter = IIR2Filter(order, [cutoff], ftype, design=design, fs=sample_rate)


""" c = 0
for i in tpp_roll_rate[0]:
    # print(tpp_roll_rate[0][c])
    tpp_roll_rate[0][c] = filter.filter(tpp_roll_rate[0][c])
    tpp_pitch_rate[0][c] = filter.filter(tpp_pitch_rate[0][c])
    c += 1


a = 0
for i in tpp_roll_raterate[0]:
    # print(tpp_roll_rate[0][c])
    tpp_roll_raterate[0][a] = filter.filter(tpp_roll_raterate[0][a])
    tpp_pitch_raterate[0][a] = filter.filter(tpp_pitch_raterate[0][a])
    a += 1 """


# b = 0
# for i in vx[0]:
#     vx[0][b] = filter.filter(vx[0][b])
#     vy[0][b] = filter.filter(vy[0][b])
#     vz[0][b] = filter.filter(vz[0][b])
#     b += 1


#print(type(tpp_roll_rate[0]))
#print(type(time))
#print(tpp_roll[0]*(180/np.pi))
#print(tpp_pitch[0]*(180/np.pi))


## generate the plot
#plt.figure(figsize=(10, 5))
fig, ((ax1,ax2), (ax3,ax4)) = plt.subplots(2, 2, figsize=(40, 10))
#print(cmd_x[0])

cmd_x[0] = ndimage.median_filter(cmd_x[0], size=1000)
cmd_y[0] = ndimage.median_filter(cmd_y[0], size=1000)
ax1.plot(time[0], cmd_x[0]/1000000, label='cmd_x', color='blue',linewidth=2)
ax1.plot(time[0], cmd_y[0]/1000000, label='cmd_y', color='red',linewidth=2)
ax1.legend()
ax1.set_title('Control signal vs time', fontsize=20)
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Cmd')

plt.subplots_adjust(hspace=0.34, wspace=0.2)

ax2.plot(time[0], np.round((tpp_roll[0]*(180/np.pi)),3), label='tpp_roll', color='blue')
ax2.plot(time[0], np.round((tpp_pitch[0]*(180/np.pi)),3), label='tpp_pitch', color='red')
ax2.legend()
ax2.set_title('TPP angle(deg) vs time', fontsize=20)
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Angle(deg)')

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

# ax4.plot(time[0], px[0], label='px', color='red')
# ax4.legend()
# ax4.set_title('px', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('m')

# ax4.plot(time[0][50:], vy[0][50:], label='vx', color='red')
# ax4.legend()
# ax4.set_title('vx', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('m/s')

# ax4.plot(time[0], ref_rates_abt_x[0], label='ref_rates_abt_x', color='blue')
# ax4.plot(time[0], ref_rates_abt_y[0], label='ref_rates_abt_y', color='red')
# ax4.legend()
# ax4.set_title('Flatness for rates', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Angle/s(rad/s)')

# ax4.plot(time[0][200:], ref_raterates_abt_x[0][200:], label='ref_raterates_abt_x', color='blue')
# ax4.plot(time[0][200:], ref_raterates_abt_y[0][200:], label='ref_raterates_abt_y', color='red')
# ax4.legend()
# ax4.set_title('Flatness for raterates', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Angle/s^2(rad/s^2)')

#ax4.plot(time[0], collective_z[0], label='Collective input', color='blue')
#ax4.plot(time[0], z_control[0], label='z_control_input', color='red')
# ax4.plot(time[0][200:], des_thrust[0][200:], label='des_thrust', color='green')
# ax4.legend()
# ax4.set_title('Collective control', fontsize=20)
# ax4.set_xlabel('Time(s)')
# ax4.set_ylabel('Signal')

ax4.plot(time[0], precession[0], label='precession', color='green')
ax4.legend()
ax4.set_title('Precession control', fontsize=20)
ax4.set_xlabel('Time(s)')
ax4.set_ylabel('Signal')


# Show the figure
plt.show()

