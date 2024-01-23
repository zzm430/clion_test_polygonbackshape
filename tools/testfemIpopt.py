import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon

testfemipopt = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testfemipopt.txt')
testfemipopt_x = testfemipopt[0]
testfemipopt_y = testfemipopt[1]

testoriginPath = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testoriginPath.txt')
testoriginPath_x = testoriginPath[0]
testoriginPath_y = testoriginPath[1]

f=plt.figure();
ax=f.add_subplot(111)
ax.plot(testfemipopt_x,testfemipopt_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
ax.plot(testoriginPath_x,testoriginPath_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

plt.axis('equal')  # 让横坐标间隔等于纵坐标间隔
f.tight_layout()
plt.show()
