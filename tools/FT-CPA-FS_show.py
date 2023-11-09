import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

fig, ax = plt.subplots()

#
C1path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA1path.txt')
C1path_x = C1path[0]
C1path_y = C1path[1]

C2path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA2path.txt')
C2path_x = C2path[0]
C2path_y = C2path[1]

C3path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA3path.txt')
C3path_x = C3path[0]
C3path_y = C3path[1]

test1007 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test1007.txt')
test1007_x = test1007[0]
test1007_y = test1007[1]

CC = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt')
CC_x = CC[0]
CC_y = CC[1]

ax.plot(CC_x,CC_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

ax.plot(C1path_x,C1path_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(C2path_x,C2path_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(C3path_x,C3path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

# for a, b in zip(CC_x,CC_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

ax.set_xlabel('x label')
ax.set_ylabel('y label')
ax.set_title('Simple Plot')
ax.set_aspect('equal')
# ax.autoscale()
# ax.legend()

# plt.xlim(0, 700) # 横坐标显示范围为0到6
# plt.ylim(0, 1500)

# mng = plt.get_current_fig_manager()
# mng.full_screen_toggle()
fig.tight_layout()

plt.show()