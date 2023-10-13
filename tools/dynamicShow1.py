import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 创建绘图对象
fig, ax = plt.subplots()

# 初始矩形位置和大小
rect = plt.Rectangle((0, 0), 1, 1, fc='blue')

# 初始化函数，用于绘制初始帧
def init():
    ax.add_patch(rect)
    return rect,

# 假设有一个包含矩形位置和姿态信息的数据集
data = [
    (0, 0, 1, 1, 0),
    (1, 0, 2, 1, 30),
    (2, 0, 1, 2, 60),
    (1, 1, 1.5, 1.5, -45),
]

# 更新函数，用于在每一帧更新矩形位置和姿态
def update(frame):
    # 根据当前帧数获取对应的数据
    x, y, width, height, angle = data[frame]

    # 更新矩形对象的位置、大小和姿态
    rect.set_xy((x, y))
    rect.set_width(width)
    rect.set_height(height)
    rect.set_angle(angle)

    return rect,


# 创建动画
ani = FuncAnimation(fig, update, frames=len(data), init_func=init, blit=True)

# 设置坐标轴范围
ax.set_xlim(-2, 3)  # 设置 x 范围为 -2 到 3
ax.set_ylim(-2, 3)  # 设置 y 范围为 -2 到 3

# 显示动画
plt.show()