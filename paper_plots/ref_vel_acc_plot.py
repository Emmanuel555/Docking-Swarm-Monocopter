import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import trajectory_generator
import statistics
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.axes3d import get_test_data


def compute_l2_norm(x, y, z):
    return np.sqrt(x**2 + y**2 + z**2)

if __name__ == "__main__":
    # circle parameters
    radius = 1.0 # 0.5
    speedX = 5.0 # 0.5 m/s the best thus far
    laps = 5
    leminiscate_laps = 4
    leminiscate_radius = 1.5
    helix_laps = 9
    alt = 1.0
    elevated_alt = 1.0
    reverse_cw = 1 # 1 for clockwise, 0 for counterclockwise

    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()
    fig = plt.figure(figsize=(20, 8))
    graphs = [[None]*3 for _ in range(1)]

    # Create 1x3 3D subplots manually
    for i in range(1):
        for j in range(3):
            ax = fig.add_subplot(1, 3, i * 3 + j + 1)
            #ax = fig.add_subplot(2, 3, i * 3 + j + 1)
            graphs[i][j] = ax

    colors = ['#254abe','#96400b','#254abe','#96400b']
    fig.subplots_adjust(hspace=0.284, wspace=0.223, 
                        left=0.04, right=0.98, 
                        top =0.87, bottom =0.043)
    fig.suptitle("ref traj vel and acc norm @ 0.5 m/s", fontsize=40, fontweight='bold')

    for a in range(len(graphs[0])): # 3D plot for trajs
        if a == 0:
            traj_label = 'Circle '
            pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(0.0, 0.0, radius, speedX, 250, laps, reverse_cw, alt) # mechanical limit for monocopter is 0.5m/s
        elif a == 1:
            traj_label = 'Elevated circle '
            pva,num_pts = traj_gen.compute_jerk_snap_9pt_elevated_circle_x_laps(0.0, 0.0, radius, speedX, 250, laps,reverse_cw,elevated_alt)
        else:
            traj_label = 'Lemniscate '
            pva,num_pts = traj_gen.lemniscate(0.0, 0.0, leminiscate_laps, leminiscate_radius, 250, reverse_cw, speedX, alt)

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

        pos_norm = []
        vel_norm = []
        acc_norm = []
        jerk_norm = []
        snap_norm = []  

        for i in range(len(pos_x)):
            pos_norm.append(compute_l2_norm(pos_x[i], pos_y[i], pos_z[i]))
            vel_norm.append(compute_l2_norm(vel_x[i], vel_y[i], vel_z[i]))
            acc_norm.append(compute_l2_norm(acc_x[i], acc_y[i], acc_z[i]))
            jerk_norm.append(compute_l2_norm(jerk_x[i], jerk_y[i], jerk_z[i]))
            snap_norm.append(compute_l2_norm(snap_x[i], snap_y[i], snap_z[i]))

        # print (traj_label + 'pos_norm', len(pos_norm))
        # smooth the data - gaussian filter inflates the data, not good for errors (median filter still better)
        vel_norm = ndimage.gaussian_filter1d(vel_norm, 200)
        acc_norm = ndimage.gaussian_filter1d(acc_norm, 200)

        for i in range(2):
            if i == 0:
                tv = np.linspace(0, len(vel_norm), len(vel_norm))
                graphs[0][a].plot(tv, vel_norm, label='vel_norm', color=colors[i]) 
            else:
                ta = np.linspace(0, len(acc_norm), len(acc_norm))
                graphs[0][a].plot(ta, acc_norm, label='acc_norm', color=colors[i])

        graphs[0][a].legend(loc='upper right', fontsize=25)
        # add title
        length = str(len(pos_norm))
        graphs[0][a].set_title(traj_label + ' @ 0.5 m/s' + length, fontsize=25, fontweight='bold')
        graphs[0][a].tick_params(axis='both', labelsize=10)     
    

plt.savefig('ref_vel_acc_norm @ 0.5 ms-1.png', dpi=300, bbox_inches='tight') 
plt.show()
    