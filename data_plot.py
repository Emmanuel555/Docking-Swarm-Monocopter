import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la

file_path = '/home/emmanuel/Monocopter-OCP/data_selected/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
position = mat_data['Monocopter_XYZ']
px = [row[0] for row in position] # column vector
py = [row[1] for row in position]
pz = [row[2] for row in position]

print('Px: ', np.size(px))
#gains = mat_data['gains']

ref_position = mat_data['ref_position']
px_des = [row[0] for row in ref_position]
py_des = [row[1] for row in ref_position]
pz_des = [row[2] for row in ref_position]


# altitude error
x_error_squared = [(px_des - px) ** 2 for px_des, px in zip(px_des, px)]
y_error_squared = [(py_des - py) ** 2 for py_des, py in zip(py_des, py)]
z_error_squared = [(pz_des - pz) ** 2 for pz_des, pz in zip(pz_des, pz)]

time_flat_list = [item for sublist in time for item in sublist]

#time_index_5 = min(range(len(time_flat_list)), key=lambda i: abs(time_flat_list[i] - 5))


max_sampling_rate = 360

start = 10*max_sampling_rate
end = 100*max_sampling_rate

rmse_num_x = x_error_squared[start:end]
rmse_num_y = y_error_squared[start:end]
rmse_num_z = z_error_squared[start:end]

final_rmse_x = math.sqrt(sum(rmse_num_x)/(end-start))
final_rmse_y = math.sqrt(sum(rmse_num_y)/(end-start))
final_rmse_z = math.sqrt(sum(rmse_num_z)/(end-start))
final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
#mean_error_squared = sum(trim_error_squared)/len(trim_error_squared)
#print(np.shape(rmse_num_x))
print('Final RMSE: ', final_rmse)



## plotting out

""" plt.figure(figsize=(10, 5))

rmse = round(rmse, 4)

text_label = 'RMSE (robot_1): {} (m)'.format(rmse)


plt.plot(time[0]*10, pz, label='measurements', color='orange')
plt.plot(time[0]*10, pz_des, label='reference', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')
# plt.title('Altitude Plot')
plt.legend()
#plt.xlim(5, 40)
plt.ylim(-1.0, 2.0)
plt.xlabel('time (s)')
plt.ylabel('altitude (m)') """


plt.figure(figsize=(10, 5))

rmse = round(final_rmse, 4)

text_label = 'Final RMSE: {} (m)'.format(rmse)

px_graph = px[start:end]
py_graph = py[start:end]
pz_graph = pz[start:end]

px_des_graph = px_des[start:end]
py_des_graph = py_des[start:end]
pz_des_graph = pz_des[start:end]

# median_filtered_px = ndimage.median_filter(px, size=200)
# median_filtered_py = ndimage.median_filter(py, size=200)

median_filtered_px = ndimage.median_filter(px_graph, size=200)
median_filtered_py = ndimage.median_filter(py_graph, size=200)

print(np.shape(median_filtered_px))
#print('ref pos: ', px_des[0], py_des[0], pz_des[0])
plt.plot(median_filtered_px, median_filtered_py, label='measurements', color='red')
plt.plot(0.0, 0.0, label='Target_point', color='blue', marker="o", markersize=20)
#plt.plot(time[0]*10, pz_des, label='reference', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')

plt.title('(Median filter: X vs Y) ' + text_label, fontsize=20)
plt.legend()
plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')

plt.text(30, 1.6, text_label, fontsize=20, ha='left', va='bottom', color='blue')


# Show the figure
plt.show()

