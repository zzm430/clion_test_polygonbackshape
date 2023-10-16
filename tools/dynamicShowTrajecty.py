import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from PIL import Image

# 创建绘制五边形的函数
def update(frame):
    vertices = [(np.cos(2*np.pi/5*i + frame*0.1), np.sin(2*np.pi/5*i + frame*0.1)) for i in range(5)]
    polygon.set_xy(vertices)
    history.append(polygon.xy.copy())  # 将当前帧的顶点坐标加入历史记录
    return (polygon,)

# 初始化图形
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)

vertices = [(np.cos(2*np.pi/5*i), np.sin(2*np.pi/5*i)) for i in range(5)]
polygon = Polygon(vertices, closed=True, fc='blue')
ax.add_patch(polygon)

history = [polygon.xy.copy()]  # 存储历史顶点坐标的列表

# 创建动画对象
ani = FuncAnimation(fig, update, frames=np.arange(100), interval=50, blit=True)

# 创建空白图像存储每一帧
blank_image = np.zeros((1, 1), dtype=np.uint8)
image_list = []

# 逐帧绘制并将图像添加到列表中
for frame in range(100):
    ani.frame_seq = frame  # 设置要绘制的帧
    fig.canvas.draw()

    # 将当前画布转换为图像并添加到列表中
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    image_list.append(image)

# 将图像列表保存为GIF
image_array = np.concatenate(image_list, axis=0)
gif_image = Image.fromarray(image_array, 'RGB')
gif_image.save('history.gif', format='GIF', append_images=[gif_image], save_all=True, duration=200, loop=0)

plt.show()