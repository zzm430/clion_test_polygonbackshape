#多个多边形展示动图
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon


# 创建绘制多边形的函数
def update(frame):
    # 更新五边形的顶点坐标
    vertices1 = [(np.cos(2 * np.pi / 5 * i + frame * 0.1), np.sin(2 * np.pi / 5 * i + frame * 0.1)) for i in range(5)]
    polygon1.set_xy(vertices1)

    # 更新六边形的顶点坐标
    vertices2 = [(np.cos(2 * np.pi / 6 * i - frame * 0.1), np.sin(2 * np.pi / 6 * i - frame * 0.1)) for i in range(6)]
    polygon2.set_xy(vertices2)

    return (polygon1, polygon2)

# 初始化图形
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)

vertices1 = [(np.cos(2 * np.pi / 5 * i), np.sin(2 * np.pi / 5 * i)) for i in range(5)]
polygon1 = Polygon(vertices1, closed=True, fc='blue')
ax.add_patch(polygon1)

vertices2 = [(np.cos(2 * np.pi / 6 * i), np.sin(2 * np.pi / 6 * i)) for i in range(6)]
polygon2 = Polygon(vertices2, closed=True, fc='green')
ax.add_patch(polygon2)

# 创建动画对象
ani = FuncAnimation(fig, update, frames=np.arange(100), interval=50, blit=True)

plt.show()