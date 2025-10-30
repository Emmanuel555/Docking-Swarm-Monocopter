import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics


data_sorter = cf.sort_traj_data()

foam_wing = 'foam_traj_data'

selected_wing = foam_wing


if selected_wing == foam_wing:
    title = 'Controller tracking performance using ultralight-wing'

# test short wing first
wing_circle = data_sorter.foam_whisker_circle(selected_wing)
wing_elevated = data_sorter.foam_whisker_elevated_circle(selected_wing)

wing = [wing_circle,wing_elevated]

## generate the plot
fig, ((ax1,ax3),(ax2,ax4)) = plt.subplots(2, 2, figsize=(50, 20))
graphs = [[ax1,ax2],[ax3,ax4]]
colors = ['#254abe','#96400b','#254abe','#96400b']
med_colors = ['#34be25','#34be25','#34be25','#34be25']
fig.subplots_adjust(hspace=0.3, wspace=0.08, 
                    left=0.048, right=0.97, 
                    top = 0.86, bottom = 0.042)
fig.suptitle(title + ' (L2 norm error(m))', fontsize=80, fontweight='bold')


for a in range(len(graphs)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        traj_label = 'Circle '
    elif a == 1:
        traj_label = 'Elevated circle '
    
    for m in range(len(graphs)): # 0.3 INDI and NDI  
        if m == 0:
            method_label = '(INDI)'
        elif m == 1:
            method_label = '(NDI)'

        for i in range(3): #xyz
            if i == 0:
                label = '∥Px∥ (m)'
            elif i == 1:
                label = '∥Py∥ (m)'
            elif i == 2:
                label = '∥Pz∥ (m)'
            method = graphs[a][m].boxplot(wing[a][m][6+i],
                                #patch_artist = True,
                                boxprops={'color':colors[m]},
                                capprops={'color':'orange'},
                                medianprops={'color':med_colors[m]},
                                whiskerprops={'color':'orange'},
                                showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

            # rmse
            graphs[a][m].plot(placement+i*0.05, wing[a][m][9][0+i], 'X', 
                              mfc = med_colors[m], mec = med_colors[m], ms = 45, label='RMS') # marker type
            # mean
            graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][6+i]), 'o', 
                              mfc = med_colors[m], mec = med_colors[m], ms = 45, label='Mean') # marker type
            
            # rotate x axis labels
            graphs[a][m].tick_params(axis='both', labelrotation=0, labelsize=50)
            graphs[a][m].set_xlim(-0.02, 0.12)
            # add title
            graphs[a][m].set_title(traj_label + method_label, fontsize=70, fontweight='bold')
            graphs[a][m].grid(linewidth=4.0)
            # add legend
            if i == 0:
                graphs[a][m].legend(loc='upper right', fontsize=50)


            for whisker in method['whiskers']:
                whisker.set(linewidth=15.0) # Set the thickness 

            for box in method['boxes']:
                box.set(linewidth=15.0) # Set the thickness 

            for cap in method['caps']:
                cap.set(linewidth=15.0) # Set the thickness
                y = cap.get_ydata()[0]

            for median in method['medians']:
                median.set(linewidth=15.0) # Set the thickness
    

""" # Iterate through each median line and its corresponding data
for i, line in enumerate(indi['medians']):
    # Get the median value
    median_value = line.get_ydata()[0]  # Both y-data points are the same on a median line

    # Get the x position of the box (boxes are evenly spaced)
    x_position = 0

    # Add the text label above the median line
    ax1.text(x_position, median_value,  # x, y coordinates for the text
            f'{median_value:.2f}',  # The text to display (formatted to 2 decimal places)
            ha='center',  # Horizontal alignment
            va='bottom',  # Vertical alignment
            fontsize=10,  # Adjust fontsize as needed
            color='black')  # Adjust color as needed
 """


plt.savefig('Reduced '+title+'.png', dpi=300, bbox_inches='tight')  
#plt.show()
