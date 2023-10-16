import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

tractorHeadPtsStream = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt')

fig, ax = plt.subplots()
ax.set_xlim(140, 155)
ax.set_ylim(15, 25)

for i in range(len(tractorHeadPtsStream) - 1):
    coords = [(tractorHeadPtsStream[i][j], tractorHeadPtsStream[i+1][j]) for j in range(4, 8)]
    polygon = Polygon(coords, closed=True, fill=False, edgecolor='green', alpha=0.9,linewidth=0.1)  # 设置透明度为 0.5
    ax.add_patch(polygon)

plt.show()