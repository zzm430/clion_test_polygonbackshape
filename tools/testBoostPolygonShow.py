import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon

testOriginPoly = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testOriginPoly.txt')
testOriginPoly_x = testOriginPoly[0]
testOriginPoly_y = testOriginPoly[1]

testFTCPACV = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testFTCPACV.txt')
testFTCPACV_x = testFTCPACV[0]
testFTCPACV_y = testFTCPACV[1]

referenceLine2 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/reference_pts2.txt')
referenceLine2_x = referenceLine2[0]
referenceLine2_y = referenceLine2[1]

searchRegionShow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/searchRegionShow.txt')
searchRegionShow_x = searchRegionShow[0]
searchRegionShow_y = searchRegionShow[1]

obstacleShow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testobstaclem.txt')
obstacleShow_x = obstacleShow[0]
obstacleShow_y = obstacleShow[1]

# test_smoothed_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_smoothed_path.txt')
# test_smoothed_path_x = test_smoothed_path[0]
# test_smoothed_path_y = test_smoothed_path[1]

pathResultShow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/pathResultShow.txt')
pathResultShow_x = pathResultShow[0]
pathResultShow_y = pathResultShow[1]

# data_trailer_file = '/home/zzm/Desktop/test_path_figure-main/src/trailer_path_test.txt'
# trailer_points = []
# with open(data_trailer_file, 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         # 提取坐标点
#         x, y = map(float, line.strip().split())
#         trailer_points.append((x, y))

# 绘制散点图
# x_values_m = [point[0] for point in trailer_points]
# y_values_m = [point[1] for point in trailer_points]

testVirtualLIne = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/testVirtualLIne.txt')
testVirtualLIne_x = testVirtualLIne[0]
testVirtualLIne_y = testVirtualLIne[1]

testfemObstaclePath = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testfemObstaclePath.txt')
testfemObstaclePath_x = testfemObstaclePath[0]
testfemObstaclePath_y = testfemObstaclePath[1]

temp_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/temp_path.txt')
temp_path_x = temp_path[0]
temp_path_y = temp_path[1]

test_temp_path1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_temp_path1.txt')
test_temp_path1_x = test_temp_path1[0]
test_temp_path1_y = test_temp_path1[1]

testObstacleOriginLine = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/testObstacleOrginLine.txt')
testObstacleOriginLine_x = testObstacleOriginLine[0]
testObstacleOriginLine_y = testObstacleOriginLine[1]

# test_boxs_show = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_boxs_show.txt')
# test_boxs_show1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_boxs_show1.txt')
test_boxs_trailer_show1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_boxs_trailer_show1.txt')

# tractorHeadPtsStream111 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/tractorObstaclesPJPO.txt')

test_obstacle_test = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_obstacle_test.txt')
test_obstacle_test_x = test_obstacle_test[0]
test_obstacle_test_y = test_obstacle_test[1]


storage_temp_stream = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/storage_temp_stream.txt')
storage_temp_stream_x = storage_temp_stream[0]
storage_temp_stream_y = storage_temp_stream[1]

storage_tractor_stream = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/storage_tractor_stream.txt')
storage_tractor_stream_x = storage_tractor_stream[0]
storage_tractor_stream_y = storage_tractor_stream[1]

with open('/home/zzm/Desktop/test_path_figure-main/src/test_PJPO_path1.txt', 'r') as file:
    lines = file.readlines()

paths = []
for i in range(0, len(lines), 2):
    x_values = list(map(float, lines[i].split()))
    y_values = list(map(float, lines[i + 1].split()))
    paths.append((x_values, y_values))

with open('/home/zzm/Desktop/test_path_figure-main/src/pathProfile1.txt','r') as file1:
    lines1 = file1.readlines()
referpaths1 = []
for m in range(0,len(lines1),2):
    x_values1 = list(map(float, lines1[m].split()))
    y_values1 = list(map(float, lines1[m+1].split()))
    referpaths1.append((x_values1, y_values1))

CC = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt')
CC_x = CC[0]
CC_y = CC[1]

# 读取txt文件中的多边形点集
with open('testOriginPolyBoost.txt') as g:
    polygon_str = g.read().strip()

# 解析多边形点集
polygon_points = polygon_str.split(',')
x_values = [float(point.split()[0]) for point in polygon_points]
y_values = [float(point.split()[1]) for point in polygon_points]

with open('/home/zzm/Desktop/test_path_figure-main/src/obstaclesShow1.txt', 'r') as file:
    lines = file.readlines()

polygons = []
for line in lines:
    vertices = line.strip().split(',')
    polygon = [(float(vertex.split()[0]), float(vertex.split()[1])) for vertex in vertices]
    polygons.append(polygon)

with open('testInsectPolygon1127.txt') as m:
    polygon_str1 = m.read().strip()

# 解析多边形点集
polygon_points1 = polygon_str1.split(',')
x_values1 = [float(point.split()[0]) for point in polygon_points1]
y_values1 = [float(point.split()[1]) for point in polygon_points1]

f=plt.figure();
ax=f.add_subplot(111)
# ax.plot(x_values,y_values,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
ax.plot(CC_x,CC_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(referenceLine2_x,referenceLine2_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3,markersize=1)
# ax.plot(testOriginPoly_x,testOriginPoly_y,color='y',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 8.3,markersize=1)
ax.plot(testObstacleOriginLine_x,testObstacleOriginLine_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
ax.plot(testVirtualLIne_x,testVirtualLIne_y,color='y',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=4)
ax.plot(testFTCPACV_x,testFTCPACV_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(searchRegionShow_x,searchRegionShow_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(obstacleShow_x,obstacleShow_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(pathResultShow_x,pathResultShow_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
ax.plot(testfemObstaclePath_x,testfemObstaclePath_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
# ax.plot(temp_path_x,temp_path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=1)
# ax.plot(test_temp_path1_x,test_temp_path1_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.scatter(x_values_m, y_values_m, color='red', s=5)             #path点位坐标展示
# ax.plot(storage_temp_stream_x,storage_temp_stream_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=1)
# ax.plot(storage_tractor_stream_x,storage_tractor_stream_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=1)

#  绘制线段
for i in range(0, len(test_obstacle_test[0])-1, 2):
    x = [test_obstacle_test[0][i], test_obstacle_test[0][i+1]]
    y = [test_obstacle_test[1][i], test_obstacle_test[1][i+1]]
    plt.plot(x, y,color='g',linewidth= 0.3,markersize=1)
    plt.text(test_obstacle_test[0][i], test_obstacle_test[1][i], f"({test_obstacle_test[0][i]}, {test_obstacle_test[1][i]})")

# for path in paths:
#     ax.plot(path[0], path[1],color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=4)
#
# for path1 in referpaths1:
#     ax.plot(path1[0], path1[1], color='g', markerfacecolor='green', marker='o', label='keypoints data', linewidth=1.3,
#                 markersize=4)

#绘制多边形
for polygon in polygons:
   x,y = zip(*polygon)
   ax.plot(x,y,marker = 'o')
   # for i, vertex in enumerate(polygon):
   #     plt.text(vertex[0], vertex[1], f'({vertex[0]}, {vertex[1]})', fontsize=8)

# for m in range(len(tractorHeadPtsStream111) - 1):
#     if m % 2 == 0:
#         coords11 = [(tractorHeadPtsStream111[m][j], tractorHeadPtsStream111[m + 1][j]) for j in range(4)]
#         coords12 = [(tractorHeadPtsStream111[m][j], tractorHeadPtsStream111[m + 1][j]) for j in range(4, 8)]
#         polygon1 = Polygon(coords12, closed=True, fill=False, edgecolor='green', alpha=0.9, linewidth=1)
#         polygonf1 = Polygon(coords11, closed=True, fill=False, edgecolor='gray', alpha=0.9, linewidth=1)
#         # 设置透明度为 0.5
#         ax.add_patch(polygon1)
#         ax.add_patch(polygonf1)

colors = ['red', 'green', 'blue', 'yellow']
# for m in range(len(test_boxs_show) - 1):
#     if m % 2 == 0:
#         coords11 = [(test_boxs_show[m][j], test_boxs_show[m + 1][j]) for j in range(4)]
#         polygonf1 = Polygon(coords11, closed=True, fill=False, edgecolor=colors[m // 2], alpha=0.9, linewidth=1)
#         # 设置透明度为 0.5
#         ax.add_patch(polygonf1)

# for m in range(len(test_boxs_show) - 1):
#     if m % 2 == 0:
#         coords11 = [(test_boxs_show[m][j], test_boxs_show[m + 1][j]) for j in range(4)]
#
#         if m // 2 < len(colors):
#             polygonf1 = Polygon(coords11, closed=True, fill=False, facecolor='yellow', alpha=0.9, linewidth=0.1)
#         else:
#             # 如果颜色不足，则循环使用颜色列表中的颜色
#             polygonf1 = Polygon(coords11, closed=True, fill=False, facecolor='yellow', alpha=0.9,
#                                 linewidth=0.1)
#
#         ax.add_patch(polygonf1)
        # 显示坐标点位
        # for coord in coords11:
        #     ax.text(coord[0], coord[1], f'({coord[0]}, {coord[1]})', fontsize=8, color='black', ha='center',
        #             va='center')

# for m in range(len(test_boxs_show1) - 1):
#     if m % 2 == 0:
#         coords111 = [(test_boxs_show1[m][j], test_boxs_show1[m + 1][j]) for j in range(4)]
#
#         if m // 2 < len(colors):
#             polygonf11 = Polygon(coords111, closed=True, fill=False, facecolor='blue', alpha=0.9, linewidth=0.1)
#         else:
#             # 如果颜色不足，则循环使用颜色列表中的颜色
#             polygonf11 = Polygon(coords111, closed=True, fill=False, facecolor='blue', alpha=0.9, linewidth=0.1)
#
#         ax.add_patch(polygonf11)

# for m in range(len(test_boxs_trailer_show1) - 1):
#     if m % 2 == 0:
#         coords112 = [(test_boxs_trailer_show1[m][j], test_boxs_trailer_show1[m + 1][j]) for j in range(4)]
#
#         if m // 2 < len(colors):
#             polygonf12 = Polygon(coords112, closed=True, fill=False, facecolor='red', alpha=0.7, linewidth=0.1)
#         else:
#             # 如果颜色不足，则循环使用颜色列表中的颜色
#             polygonf12 = Polygon(coords112, closed=True, fill=False, facecolor='red', alpha=0.7,
#                                 linewidth=0.1)
#         ax.add_patch(polygonf12)

# ax.plot(referenceLine2_x,referenceLine2_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)

print("x values:", x_values)
print("y values:", y_values)

# for a, b in zip(test_temp_path1_x,test_temp_path1_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

plt.axis('equal')  # 让横坐标间隔等于纵坐标间隔

f.tight_layout()
plt.show()