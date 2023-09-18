import matplotlib.pyplot as plt
import numpy as np

# 定义起点和终点坐标
x1_start = [1, 3, 2]
y1_start = [1, 4, 2]
x1_end = [5, 6, 4]
y1_end = [3, 2, 5]

x2_start = [1, 4, 2]
y2_start = [3, 2, 5]
x2_end = [5, 6, 4]
y2_end = [3, 4, 2]

# 创建画布和子图
fig, ax = plt.subplots()

# 绘制两条线段
ax.plot(x1_start, y1_start, 'r-', label='Line 1')
ax.plot(x2_start, y2_start, 'b-', label='Line 2')

# 填充颜色
x_fill = np.linspace(1, 5, 100)
y_min = np.minimum(np.interp(x_fill, x1_start, y1_start), np.interp(x_fill, x2_start, y2_start))
y_max = np.maximum(np.interp(x_fill, x1_end, y1_end), np.interp(x_fill, x2_end, y2_end))
ax.fill_between(x_fill, y_min, y_max, color='gray', alpha=0.5)

# 可选：设置坐标轴范围
ax.set_xlim(0, 7)
ax.set_ylim(0, 6)

# 添加图例
ax.legend()

# 可选：添加标题和标签
ax.set_title('Filled Area between Two Lines')
ax.set_xlabel('X')
ax.set_ylabel('Y')

# 显示图形
plt.show()