
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
lineshow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeaveline.txt')


# 绘制线段
for i in range(0, len(lineshow[0])-1, 2):
    x = [lineshow[0][i], lineshow[0][i+1]]
    y = [lineshow[1][i], lineshow[1][i+1]]
    plt.plot(x, y)
    plt.text(lineshow[0][i], lineshow[1][i], f"({lineshow[0][i]}, {lineshow[1][i]})")

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