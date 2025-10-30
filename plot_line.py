import numpy as np
import matplotlib.pyplot as plt
import minsnap_trajectories as ms
import trajectory_generator
from pyrr import quaternion
import timeit
import copy
from random import randrange
import matplotlib.animation as animation
from matplotlib import style
from mpl_toolkits.mplot3d import Axes3D

alt = 1.2
start = timeit.default_timer() 

y = [0.0,0.0,0.0,0.0,0.0]
x = [1.5,0.5,0.0,-0.5,-1.5]
parts = len(x)
distance = abs(x[0])*2
speed = 0.1
total_time = distance/speed
sampling_freq = 360
num_pts = int(total_time*sampling_freq)
refs = []

for i in range(parts):
    refs.append(ms.Waypoint(
        time=(total_time/(parts-1))*i,
        position=np.array([x[i], y[i], alt]), # altitude used to be 1.0
    ))


polys = ms.generate_trajectory(
        refs,
        degree=8,  # Polynomial degree
        idx_minimized_orders=(3, 4),  
        num_continuous_orders=3,  
        algorithm="closed-form",  # Or "constrained"
    )


# t = np.linspace(0, total_time, num_pts)
# ## Sample up to the 3rd order (Jerk) -----v
# pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6

# ## available order in derivatives in pva is one lesser than computed meaning up to one derivative less  
# print(np.shape(pva)) # up to order of derivatives available, waypoints, axis
# pos_x = pva[0,:,0] # position
# vel_x = pva[1,:,0] # velocity
# acc_x = pva[2,:,0] # accleleration
# jerk_x = pva[3,:,0] # jerk
# snap_x = pva[4,:,0] # snap

# pos_y = pva[0,:,1] # position
# vel_y = pva[1,:,1] # velocity
# acc_y = pva[2,:,1] # acceleration
# jerk_y = pva[3,:,1] # jerk
# snap_y = pva[4,:,1] # snap


traj_gen = trajectory_generator.trajectory_generator()
#derivatives = traj_gen.jerk_snap_9pt_circle(0, 0.4, 0, 1)
#pva,num_pts = traj_gen.lemniscate(0.0, 0.0, 2, 1.3, 250, 1, 2, alt)
pva,num_pts = traj_gen.compute_jerk_snap_9pt_elevated_circle_x_laps(0.0, 0.0, 1.0, 5.0, 250, 3, 0, 1.0)
    
#pva,num_pts = traj_gen.lemniscate(0.0, 0.0, 2, 1.3, 250, 1, 2, alt)
#pva,num_pts = traj_gen.two_pt_line(1,250,alt)
#pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(1.16,-0.29,0.9,1,5,1)

#derivatives = traj_gen.jerk_snap_circle(0.5,0,0,1) .....
pos_x = pva[0,:,0] # position
vel_x = pva[1,:,0] # velocity
acc_x = pva[2,:,0] # accleleration
jerk_x = pva[3,:,0] # jerk
snap_x = pva[4,:,0] # snap

pos_y = pva[0,:,1] # position
vel_y = pva[1,:,1] # velocity
acc_y = pva[2,:,1] # acceleration
jerk_y = pva[3,:,1] # jerk
snap_y = pva[4,:,1] # snap

pos_z = pva[0,:,2] # position
vel_z = pva[1,:,2] # velocity   
acc_z = pva[2,:,2] # acceleration
jerk_z = pva[3,:,2] # jerk
snap_z = pva[4,:,2] # snap

#t = np.linspace(0, total_time, num_pts)
t = np.linspace(0, np.size(pos_z), np.size(pos_z))

#print (len(derivatives))
#print(np.shape(pos))
fig, ax = plt.subplots(1)
# ax.plot(t, pos_x, label='pos_x', color='blue')
# ax.plot(t, jerk_x, label='jerk_x', color='red')
# ax.plot(t, snap_x, label='snap_x', color='green')
# ax.set_title('Derivatives gen', fontsize=20)
#ax.plot(pos_x, pos_y, label='circle', color='blue')
ax.plot(t, pos_z, label='lemniscate', color='blue')
#ax.set_title('Derivatives gen', fontsize=20)
ax.set_title('lemniscate', fontsize=20)
ax.legend()
#ax.set_xlabel('Time(1/250s)')
#ax.set_ylabel('X axis input')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
plt.show()





    
        

#stop = timeit.default_timer()
#print('Program Runtime: ', stop - start)  