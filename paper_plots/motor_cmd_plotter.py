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

selected_wing = long_wing

if selected_wing == short_wing:
    title = 'Short-wing '
elif selected_wing == long_wing:
    title = 'Long-wing '

wing_circle = data_sorter.whisker_circle(selected_wing)
wing_elevated = data_sorter.whisker_elevated_circle(selected_wing)
wing_lem = data_sorter.whisker_lem(selected_wing)

wing = [wing_circle,wing_elevated,wing_lem]


## generate the plot
fig, ((ax1,ax2,ax3)) = plt.subplots(1, 3, figsize=(20, 8))
graphs = [ax1,ax2,ax3]
#colors = ['#254abe','#96400b','#a725be']
colors = ['#254abe','#96400b']
fig.subplots_adjust(hspace=0.3, wspace=0.4, 
                    left=0.048, right=0.97, 
                    top = 0.85, bottom = 0.077)
fig.suptitle(title+'Motor PWM Commands', fontsize=40, fontweight='bold')


for a in range(len(wing)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        traj_label = 'Circle '
    elif a == 1:
        traj_label = 'Elevated circle '
    elif a == 2:
        traj_label = 'Lemniscate '


    for i in range(2): # 2 methods INDI vs NDI
        # Only for traj 0.5m/s
        if i == 0:
            method_label = 'INDI '
        elif i == 1:
            method_label = 'NDI '

        # motor
        start = wing[a][i+3][11][0]
        traj_time_motor_5 = wing[a][i+3][11] - start
        motor_cmd_5 = wing[a][i+3][13]   
        
        graphs[a].plot(traj_time_motor_5, motor_cmd_5, color=colors[i], linestyle='solid', linewidth=3.0, label=method_label) 
        graphs[a].set_title(traj_label, fontsize=30, fontweight='bold')

ax1.set_xlabel('Time(s)', fontsize=25)
ax1.set_ylabel('pwm', fontsize=25)
ax2.set_xlabel('Time(s)', fontsize=25)
ax2.set_ylabel('pwm', fontsize=25)
ax3.set_xlabel('Time(s)', fontsize=25)
ax3.set_ylabel('pwm', fontsize=25)
ax1.tick_params(axis='both', labelsize=25)
ax2.tick_params(axis='both', labelsize=25)
ax3.tick_params(axis='both', labelsize=25)
ax1.legend(loc='upper right', fontsize=25)
ax2.legend(loc='upper right', fontsize=25)
ax3.legend(loc='upper right', fontsize=25)
#ax2.set_title('Long wing motor commands @ cir 0.3m/s', fontsize=30, fontweight='bold')
#ax3.set_title('Long wing motor commands @ cir 0.5m/s', fontsize=30, fontweight='bold')
ax1.grid(linewidth=2.0)
ax2.grid(linewidth=2.0)
ax3.grid(linewidth=2.0)    
  
plt.savefig(title+'motor_cmd.png', dpi=300, bbox_inches='tight')  
plt.show()
