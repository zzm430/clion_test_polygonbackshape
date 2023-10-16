#对应的txt的规则
#每两行为一对 前4列为第一个多边形A  后4列为第二个多边形B
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

tractorHeadPtsStream = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt')

coordinates = []
frame_count = len(tractorHeadPtsStream) - 1  # 帧数减去最后一帧
duration = 1000  # 动画总持续时间（毫秒）
interval = duration / frame_count  # 每个图形显示的时间间隔（毫秒）

# 创建绘制五边形的函数
def update(frame):
    # vertices =  [(np.cos(2*np.pi/5*i + frame*0.1), np.sin(2*np.pi/5*i + frame*0.1)) for i in range(5)]
    if frame % 2 == 0:  # 只在 frame 为偶数时执行操作
        if frame < len(tractorHeadPtsStream) - 1:
             vertices = [(tractorHeadPtsStream[frame][i], tractorHeadPtsStream[frame+1][i]) for i in range(4)]
             vertices2 = [(tractorHeadPtsStream[frame][i], tractorHeadPtsStream[frame + 1][i]) for i in range(4, 8)]
             polygon.set_xy(vertices)
             polygon2.set_xy(vertices2)
    return (polygon,polygon2)

# 初始化图形
fig, ax = plt.subplots()
ax.set_xlim(-300, 300)
ax.set_ylim(-300, 300)
# ax.autoscale()
vertices = [(tractorHeadPtsStream[0][i], tractorHeadPtsStream[1][i]) for i in range(4)]
polygon = Polygon(vertices, closed=True, fc='blue')
ax.add_patch(polygon)

vertices2 = [(tractorHeadPtsStream[0][i], tractorHeadPtsStream[1][i])for i in range(4, 8)]
polygon2 = Polygon(vertices2, closed=True, fc='blue')
ax.add_patch(polygon2)

# 创建动画对象
# ani = FuncAnimation(fig, update, frames=np.arange(100), interval=interval,repeat=True, blit=True)
ani = FuncAnimation(fig, update, frames=np.arange(frame_count), interval=interval,repeat=True, blit=True)

plt.show()

# 在上述代码中，通过将 repeat=True 传递给 FuncAnimation，设置动画为循环播放。这样，当所有帧执行完毕后，动画会重新开始，并以循环的方式继续播放。
#
# 请注意，frames=np.arange(100) 表示只选择前 100 帧来生成动画。如果您希望使用所有帧，请将其修改为 frames=np.arange(frame_count)。