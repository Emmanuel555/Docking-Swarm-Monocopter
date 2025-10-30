import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
from mpl_toolkits.mplot3d import Axes3D



data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
#short_wing = 'short_traj_data'

selected_wing = long_wing
att = 1

# test short wing first
wing_circle = data_sorter.plot_circle(selected_wing,att)
wing_elevated = data_sorter.plot_elevated_circle(selected_wing,att)
wing_lem = data_sorter.plot_lem(selected_wing,att)

wing = [wing_circle,wing_elevated,wing_lem]

## generate the plot
fig = plt.figure(figsize=(40, 12))
graphs = [[None]*3 for _ in range(2)]

# Create 1x3 3D subplots manually
for i in range(2):
    for j in range(3):
        ax = fig.add_subplot(2, 3, i * 3 + j + 1)
        #ax = fig.add_subplot(2, 3, i * 3 + j + 1)
        graphs[i][j] = ax

colors = ['#254abe','#96400b','#254abe','#96400b']
fig.subplots_adjust(hspace=0.248, wspace=0.118, 
                    left=0.03, right=0.98, 
                    top =0.9, bottom =0.075)
#fig.suptitle(title, fontsize=40, fontweight='bold')


for t in range(3): # trajectories
    if t == 0:
        traj_label = 'Circle '
    elif t == 1:
        traj_label = 'Elevated Circle '
    elif t == 2:
        traj_label = 'Lemniscate '   

    for a in range(2): # 2 graphs per traj
        if a == 0:
            att_label = '∥˙ωDE∥ (rad/s^2)'   
        elif a == 1:
            att_label = 'Body yaw rotational rate (Hz)'  

        if a == 1:
            for m in range(2): # 0.5 INDI and NDI  
                # only test on 0.5m/s
                m = m + 2
                if m == 2:
                    method_label = 'INDI '
                    style = 'solid'
                elif m == 3:
                    method_label = 'NDI '
                    style = 'solid'
                    #graphs[a][t].axvline(x=wing[t][m][0][-1], color='red', linestyle='--', linewidth=7.0, label='0.5m/s traj end time')

                start = wing[t][m][0][0]
                traj_time = wing[t][m][0] - start 
                yawrate = wing[t][m][11] # 9 = roll, 10 = pitch, 11 = yawrate
                mean = statistics.mean(yawrate)
                mean = np.round(mean,1)
                graphs[a][t].set_xlabel('Time(s)',fontsize=20) 
                graphs[a][t].plot(traj_time,yawrate,label=method_label+"(Mean:{})".format(mean),linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']

        else:
            for m in range(2): # 0.5 INDI and NDI  
                # only test on 0.5m/s
                m = m + 2
                if m == 2:
                    method_label = 'INDI '
                    style = 'solid'
                elif m == 3:
                    method_label = 'NDI '
                    style = 'solid'
                    #graphs[a][t].axvline(x=wing[t][m][0][-1], color='red', linestyle='--', linewidth=7.0, label='0.5m/s traj end time')

                start = wing[t][m][0][0]
                traj_time = wing[t][m][0] - start  
                att_track = wing[t][m][10] # 9 = roll, 10 = pitch, 11 = yawrate 
                mean = statistics.mean(wing[t][m][10])
                mean = np.round(mean,2)
                rms = np.round(wing[t][m][12],2)
                graphs[a][t].set_xlabel('Time(s)',fontsize=20) 
                graphs[a][t].plot(traj_time,att_track,label=method_label+"(Mean:{}, RMS:{})".format(mean, rms),linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']
                                

        # legend        
        graphs[a][t].legend(loc='upper right', fontsize=20)
        # axes
        graphs[a][t].tick_params(axis='both', labelsize=20)
        # add title
        graphs[a][t].set_title(traj_label+att_label, fontsize=25, fontweight='bold')
        graphs[a][t].grid(linewidth=4.0)



#plt.savefig(title+'.pdf')   
plt.savefig('long_wing_att.png', dpi=300, bbox_inches='tight')  
plt.show()
