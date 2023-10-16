# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
#
# with open('/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt') as f:
#     x_data = f.readline().split()
#     y_data = f.readline().split()
#     # x2_data = f.readline().split()
#     # y2_data = f.readline().split()
#
# x_data = np.array(x_data).astype('float32')
# y_data = np.array(y_data).astype('float32')
#
# coordinates = np.column_stack((x_data, y_data))
# polygon = Polygon(coordinates, closed=True, fc='orange')
#
# fig, ax = plt.subplots()
# ax.add_patch(polygon)
# ax.autoscale()
# plt.show()
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

with open('/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt') as f:
    data = f.readlines()

coordinates = []
for i in range(len(data)//2):
    x_data = data[2*i].split()
    y_data = data[2*i+1].split()
    x_data = np.array(x_data).astype('float32')
    y_data = np.array(y_data).astype('float32')
    coordinates.append(np.column_stack((x_data, y_data)))

polygon = Polygon(np.concatenate(coordinates), closed=True, fc='orange')

fig, ax = plt.subplots()
ax.add_patch(polygon)
ax.autoscale()
plt.show()