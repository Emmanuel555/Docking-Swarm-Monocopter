import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.axes3d import get_test_data


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
#short_wing = 'short_traj_data'

selected_wing = long_wing
att = 0

# test short wing first
wing_circle = data_sorter.plot_circle(selected_wing,att)
wing_elevated = data_sorter.plot_elevated_circle(selected_wing,att)
wing_lem = data_sorter.plot_lem(selected_wing,att)

wing = [wing_circle,wing_elevated,wing_lem]

## generate the plot
fig = plt.figure(figsize=(20, 8))
graphs = [[None]*3 for _ in range(1)]

# Create 1x3 3D subplots manually
for i in range(1):
    for j in range(3):
        ax = fig.add_subplot(1, 3, i * 3 + j + 1, projection='3d')
        #ax = fig.add_subplot(2, 3, i * 3 + j + 1)
        graphs[i][j] = ax

colors = ['#254abe','#96400b','#254abe','#96400b']
fig.subplots_adjust(hspace=0.3, wspace=0.19, 
                    left=0.0, right=0.94, 
                    top =1.0, bottom =0.043)
#fig.suptitle(title, fontsize=40, fontweight='bold')


for a in range(len(graphs[0])): # 3D plot for trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        traj_label = 'Circle '
    elif a == 1:
        traj_label = 'Elevated circle '
    elif a == 2:
        traj_label = 'Lemniscate '   

    for m in range(2): # 0.5 INDI and NDI  
        m = m + 2
        if m == 2:
            method_label = 'INDI '
            style = 'solid'
        elif m == 3:
            method_label = 'NDI '
            style = 'solid'

        if m == 2:
            rx = wing[a][m][4] 
            ry = wing[a][m][5] 
            rz = wing[a][m][6] 
            graphs[0][a].plot3D(rx,ry,rz,label='Ref',color='red',linewidth=7.0,linestyle='solid') 
    
        x = wing[a][m][1] 
        y = wing[a][m][2] 
        z = wing[a][m][3]
        graphs[0][a].plot3D(x,y,z,label=method_label,linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']
                
    graphs[0][a].set_zlim(0,2)
    graphs[0][a].set_xlabel('Xw [m]', fontsize=20)
    graphs[0][a].set_ylabel('Yw [m]', fontsize=20)
    graphs[0][a].set_zlabel('Zw [m]', fontsize=20)
    # legend        
    graphs[0][a].legend(loc='upper right', fontsize=23)
    # add title
    graphs[0][a].set_title(traj_label, fontsize=35, fontweight='bold')
    graphs[0][a].tick_params(axis='both', labelsize=20)
    graphs[0][a].tick_params(axis='x', pad=30)
    graphs[0][a].tick_params(axis='y', pad=30)
    graphs[0][a].tick_params(axis='z', pad=33)    



#plt.savefig(title+'.pdf')   
plt.savefig('ref_3D.png', dpi=300, bbox_inches='tight')  
plt.show()
