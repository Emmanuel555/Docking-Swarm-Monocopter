import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
foam_wing = 'foam_traj_data'


# test short wing first 11 12 13
wing_circle_long = data_sorter.whisker_circle(long_wing)
wing_circle_short = data_sorter.whisker_circle(short_wing)
wing_circle_foam = data_sorter.foam_whisker_circle(foam_wing)

wing = [wing_circle_long,wing_circle_short,wing_circle_foam]

## generate the plot
fig, ((ax1)) = plt.subplots(1, 1, figsize=(20, 8))
graphs = [ax1]
colors = ['#254abe','#96400b','#a725be']
fig.subplots_adjust(hspace=0.3, wspace=0.2, 
                    left=0.048, right=0.97, 
                    top = 0.89, bottom = 0.042)
#fig.suptitle(title + ' (L2 norm error(m))', fontsize=40, fontweight='bold')


for a in range(len(wing)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        wing_label = 'Long '
        method_label = 'INDI '
    elif a == 1:
        wing_label = 'Short '
        method_label = 'NDI '
    elif a == 2:
        wing_label = 'Ultralight ' 
        method_label = 'ATT '

    # Only for Circle 0.3m/s

    # yawrate
    traj_time_yawrate = wing[a][0][11]
    yawrate = wing[a][0][12]
    ax1.plot(traj_time_yawrate, yawrate, color=colors[a], linestyle='solid', linewidth=3.0, label=wing_label)
    
ax1.set_xlabel('Time(s)', fontsize=25)
ax1.set_ylabel('Hz', fontsize=25)
ax1.tick_params(axis='both', labelsize=25)
ax1.legend(loc='upper right', fontsize=25)
ax1.set_title('Body yaw rotational rate @ cir 0.3m/s', fontsize=30, fontweight='bold')
ax1.grid(linewidth=2.0)    
  
plt.savefig('yawrate.png', dpi=300, bbox_inches='tight')  
plt.show()
