import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon

testOriginPoly = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testOriginPoly.txt')
testOriginPoly_x = testOriginPoly[0]
testOriginPoly_y = testOriginPoly[1]

referenceLine2 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/referenceLine2.txt')
referenceLine2_x = referenceLine2[0]
referenceLine2_y = referenceLine2[1]

testVirtualLIne = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/testVirtualLIne.txt')
testVirtualLIne_x = testVirtualLIne[0]
testVirtualLIne_y = testVirtualLIne[1]

testfemSmooth = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testfemSmooth.txt')
testfemSmooth_x = testfemSmooth[0]
testfemSmooth_y = testfemSmooth[1]

testObstacleOriginLine = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/testObstacleOrginLine.txt')
testObstacleOriginLine_x = testObstacleOriginLine[0]
testObstacleOriginLine_y = testObstacleOriginLine[1]

tractorHeadPtsStream111 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/tractorObstaclesPJPO.txt')

test_PJPO_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_PJPO_path.txt')
test_PJPO_path_x = test_PJPO_path[0]
test_PJPO_path_y = test_PJPO_path[1]

# 读取txt文件中的多边形点集
with open('testOriginPolyBoost.txt') as g:
    polygon_str = g.read().strip()

# 解析多边形点集
polygon_points = polygon_str.split(',')
x_values = [float(point.split()[0]) for point in polygon_points]
y_values = [float(point.split()[1]) for point in polygon_points]


with open('testInsectPolygon1127.txt') as m:
    polygon_str1 = m.read().strip()

# 解析多边形点集
polygon_points1 = polygon_str1.split(',')
x_values1 = [float(point.split()[0]) for point in polygon_points1]
y_values1 = [float(point.split()[1]) for point in polygon_points1]

f=plt.figure();
ax=f.add_subplot(111)
ax.plot(x_values,y_values,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(testfemSmooth_x,testfemSmooth_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1,markersize=1)

ax.plot(testOriginPoly_x,testOriginPoly_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(testObstacleOriginLine_x,testObstacleOriginLine_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(testVirtualLIne_x,testVirtualLIne_y,color='y',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=4)
ax.plot(test_PJPO_path_x,test_PJPO_path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=4)

# for m in range(len(tractorHeadPtsStream111) - 1):
#     if m % 2 == 0:
#         coords11 = [(tractorHeadPtsStream111[m][j], tractorHeadPtsStream111[m + 1][j]) for j in range(4)]
#         coords12 = [(tractorHeadPtsStream111[m][j], tractorHeadPtsStream111[m + 1][j]) for j in range(4, 8)]
#         polygon1 = Polygon(coords12, closed=True, fill=False, edgecolor='green', alpha=0.9, linewidth=1)
#         polygonf1 = Polygon(coords11, closed=True, fill=False, edgecolor='gray', alpha=0.9, linewidth=1)
#         # 设置透明度为 0.5
#         ax.add_patch(polygon1)
#         ax.add_patch(polygonf1)

# ax.plot(referenceLine2_x,referenceLine2_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)

print("x values:", x_values)
print("y values:", y_values)

for a, b in zip(testObstacleOriginLine_x,testObstacleOriginLine_y):
        plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

plt.axis('equal')  # 让横坐标间隔等于纵坐标间隔

f.tight_layout()
plt.show()