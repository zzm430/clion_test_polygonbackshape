#第一列为横坐标，第二列为纵坐标
from shapely.geometry import LineString
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

# 打开并读取文本文件
with open('/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt', 'r') as file:
    lines = file.readlines()  # 读取所有行的数据

# 解析数据并提取坐标
coordinates = []
for line in lines:
    x, y = line.strip().split()  # 去除换行符并按空格分隔
    coordinates.append((float(x), float(y)))

# 创建 LineString 对象
line = LineString(coordinates)

# 计算缓冲区对象
buffered_line = line.buffer(2, cap_style="square")

fig, ax = plt.subplots(figsize=(5, 5))

# 创建缓冲区对象的多边形，并添加到图形中
patch = Polygon(buffered_line.exterior.coords, facecolor='blue', alpha=0.5)
ax.add_patch(patch)

# 绘制原始线对象
ax.plot(*line.xy)
ax.set_aspect('equal')
plt.show()