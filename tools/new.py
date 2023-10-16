import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

tractorHeadPtsStream = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt')
tractorHeadPtsStream = tractorHeadPtsStream[0]
tractorHeadPtsStream = tractorHeadPtsStream[1]

# 创建绘制五边形的函数
def update(frame):
    vertices = [(np.cos(2*np.pi/5*i + frame*0.1), np.sin(2*np.pi/5*i + frame*0.1)) for i in range(5)]
    polygon.set_xy(vertices)
    return (polygon,)

# 初始化图形
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)

vertices = [(np.cos(2*np.pi/5*i), np.sin(2*np.pi/5*i)) for i in range(5)]
polygon = Polygon(vertices, closed=True, fc='blue')
ax.add_patch(polygon)

# 创建动画对象
ani = FuncAnimation(fig, update, frames=np.arange(100), interval=50, blit=True)

plt.show()