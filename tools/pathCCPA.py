import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
CCPA1path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA1path.txt')
CCPA1path_x = CCPA1path[0]
CCPA1path_y = CCPA1path[1]
CCPA2path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA2path.txt')
CCPA2path_x = CCPA2path[0]
CCPA2path_y = CCPA2path[1]
CCPA3path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA3path.txt')
CCPA3path_x = CCPA3path[0]
CCPA3path_y = CCPA3path[1]

ax.plot(CCPA1path_x,CCPA1path_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(CCPA2path_x,CCPA2path_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

ax.plot(CCPA3path_x,CCPA3path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

ax.set_xlabel('x label')
ax.set_ylabel('y label')
ax.set_title('Simple Plot')
ax.set_aspect('equal')
# ax.autoscale()
ax.legend()

# plt.xlim(0, 700) # 横坐标显示范围为0到6
# plt.ylim(0, 1500)

# mng = plt.get_current_fig_manager()
# mng.full_screen_toggle()
fig.tight_layout()

plt.show()