#/usr/bin/python3
#功能导入txt文件中的坐标点并显示其数据

import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
# BB = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/origin_polygon.txt')
# BB_x = BB[0]
# BB_y = BB[1]
# ff = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/narrow_points.txt')
# ff_x = ff[0]
# ff_y = ff[1]
# AA = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/inter_nodes.txt')
# AA_x = AA[0]
# AA_y = AA[1]

CC = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt')
CC_x = CC[0]
CC_y = CC[1]
# DD = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/realline.txt')
# DD_x = DD[0]
# DD_y = DD[1]
# EE = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/increaseNodes.txt')
# EE_x = EE[0]
# EE_y = EE[1]
# NN = np.loadtxt('/home/zzm/Desktop/reeds_shepp-master/RS_Lib/show_ridge_path11.txt')
# NN_x = NN[0]
# NN_y = NN[1]
# test = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test0620.txt')
# test_x = test[0]
# test_y = test[1]

# test_111_poly = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_111_poly.txt')
# test_111_poly_x = test_111_poly[0]
# test_111_poly_y = test_111_poly[1]

test_virtual_origin_poly = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_virtual_origin_poly_common.txt')
test_virtual_origin_poly_x = test_virtual_origin_poly[0]
test_virtual_origin_poly_y = test_virtual_origin_poly[1]


# testAB = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testAB.txt')
# testAB_x = testAB[0]
# testAB_y = testAB[1]


test1 = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/test1.txt')
test1_x = test1[0]
test1_y = test1[1]

# testABtransd = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testABtransd.txt')
# testABtransd_x = testABtransd[0]
# testABtransd_y = testABtransd[1]

# test_move_pts = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_move_pts.txt')
# test_move_pts_x = test_move_pts[0]
# test_move_pts_y = test_move_pts[1]

# cgal_pts_entrance = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/cgal_pts_entrance.txt')
# cgal_pts_entrance_x = cgal_pts_entrance[0]
# cgal_pts_entrance_y = cgal_pts_entrance[1]

# test_skeleton_2 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_3.txt')
# test_skeleton_2_x = test_skeleton_2[0]
# test_skeleton_2_y = test_skeleton_2[1]

# test_skeleton_4 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_4.txt')
# test_skeleton_4_x = test_skeleton_4[0]
# test_skeleton_4_y = test_skeleton_4[1]

test_skeleton_6 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_6.txt')
test_skeleton_6_x = test_skeleton_6[0]
test_skeleton_6_y = test_skeleton_6[1]

test_virtual_origin_poly = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_virtual_origin_poly.txt')
test_virtual_origin_poly_x = test_virtual_origin_poly[0]
test_virtual_origin_poly_y = test_virtual_origin_poly[1]

# test_move_pts = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test_move_pts_B.txt')
# test_move_pts_x = test_move_pts[0]
# test_move_pts_y = test_move_pts[1]

inner_skeleton_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/inner_skeleton_path.txt')
inner_skeleton_path_x = inner_skeleton_path[0]
inner_skeleton_path_y = inner_skeleton_path[1]

a = np.loadtxt('/home/zzm/clion_test_polygonbackshape/tools/test.txt')
# a = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/ptsshow.txt')
# lineshow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/lineshow.txt')
# lineshow_x = lineshow[0]
# lineshow_y = lineshow[1]

# # 从txt文件读取坐标点
# with open("/home/zzm/Desktop/test_path_figure-main/src/lineshow.txt", "r") as f:
#     lines = f.readlines()
#
# points = []
# for line in lines:
#     x, y = line.strip().split(" ")
#     points.append((float(x), float(y)))
#
# # 将坐标点分成线段
# segments = [(points[i], points[i+1]) for i in range(0, len(points)-1, 2)]
#
# # 绘制线段
# for segment in segments:
#     x_values = [segment[0][0], segment[1][0]]
#     y_values = [segment[0][1], segment[1][1]]
#     plt.plot(x_values, y_values,color='black',linewidth= 0.3,markersize=1)

entrance_lines = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/cgal_pts_entrance.txt')
entrance_lines_x = entrance_lines[0]
entrance_lines_y = entrance_lines[1]

convexHull = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/convexHull.txt')
convexHull_x = convexHull[0]
convexHull_y = convexHull[1]


CCPA4path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA4path111.txt')
CCPA4path_x = CCPA4path[0]
CCPA4path_y = CCPA4path[1]

CCPA4border = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA4border.txt')
CCPA4border_x = CCPA4border[0]
CCPA4border_y = CCPA4border[1]

# C1path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/C1path.txt')
# C1path_x = C1path[0]
# C1path_y = C1path[1]
#
# C2path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/C2path.txt')
# C2path_x = C2path[0]
# C2path_y = C2path[1]
#
# C3path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/C3path.txt')
# C3path_x = C3path[0]
# C3path_y = C3path[1]
#
CCPA1path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA1path.txt')
CCPA1path_x = CCPA1path[0]
CCPA1path_y = CCPA1path[1]
CCPA2path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA2path.txt')
CCPA2path_x = CCPA2path[0]
CCPA2path_y = CCPA2path[1]
CCPA3path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA3path.txt')
CCPA3path_x = CCPA3path[0]
CCPA3path_y = CCPA3path[1]

# CCPA1path1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA1border.txt')
# CCPA1path1_x = CCPA1path1[0]
# CCPA1path1_y = CCPA1path1[1]
# CCPA2path1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA2border.txt')
# CCPA2path1_x = CCPA2path1[0]
# CCPA2path1_y = CCPA2path1[1]
# CCPA3path1 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/CCPA3border.txt')
# CCPA3path1_x = CCPA3path1[0]
# CCPA3path1_y = CCPA3path1[1]

lineshow = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeaveline.txt')

test1007 = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test1007.txt')
test1007_x = test1007[0]
test1007_y = test1007[1]

# ax.plot(test1007_x,test1007_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)

ax.plot(CCPA1path_x,CCPA1path_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.1,markersize=1)
ax.plot(CCPA2path_x,CCPA2path_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.1,markersize=1)
ax.plot(CCPA3path_x,CCPA3path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.1,markersize=1)

ax.plot(CCPA4path_x,CCPA4path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.1,markersize=1)

# ax.plot(CCPA1path1_x,CCPA1path1_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(CCPA2path1_x,CCPA2path1_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(CCPA3path1_x,CCPA3path1_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

# cgal_show_ridge_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt')
# cgal_show_ridge_path_x = cgal_show_ridge_path[0]
# cgal_show_ridge_path_y = cgal_show_ridge_path[1]
# ax.plot(ff_x,ff_y,color='b',markerfacecolor='green',marker='o',label='narrow_points data')
# ax.plot(EE_x,EE_y,color='b',markerfacecolor='green',marker='o',label='increaseNodes data')
# ax.plot(EE_x,EE_y,color='b',markerfacecolor='green',marker='o',label='increaseNodes data')
# ax.plot(DD_x,DD_y,color='y',markerfacecolor='green',marker='o',label='inter_nodes data')
# ax.plot(BB_x,BB_y,color='r',markerfacecolor='green',marker='o',label='origin_polygon data')
# ax.plot(test1_x,test1_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 4.3,markersize=1)

ax.plot(CC_x,CC_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

ax.plot(CCPA4border_x,CCPA4border_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
# ax.plot(C1path_x,C1path_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(C2path_x,C2path_y,color='b',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(C3path_x,C3path_y,color='r',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(testAB_x,testAB_y,color='black',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)
# ax.plot(testABtransd_x,testABtransd_y,color='red',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1.3,markersize=1)

# ax.plot(convexHull_x,convexHull_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

for i in range(len(a)):
    plt.plot(a[i,0],a[i,1],'ro', markersize=3)
    # plt.text(a[i, 0] + 0.2, a[i, 1] + 0.2, f"({a[i, 0]}, {a[i, 1]})")
plt.plot(a[0,0],a[0,1],'ro')

# ax.plot(test_txt_x,test_txt_y,color='black',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.8,markersize=1)
# ax.plot(entrance_lines_x,entrance_lines_y,color='black',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=1)

# ax.plot(test_skeleton_4_x,test_skeleton_4_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(test_skeleton_6_x,test_skeleton_6_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

# ax.plot(test_skeleton_2_x,test_skeleton_2_y,color='black',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 2.3,markersize=1)

# ax.plot(inner_skeleton_path_x,inner_skeleton_path_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

# ax.plot(cgal_show_ridge_path_x,cgal_show_ridge_path_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.5,markersize=1)

# ax.plot(test_virtual_origin_poly_x,test_virtual_origin_poly_y,color='red',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.5,markersize=1)
# ax.plot(test_move_pts_x,test_move_pts_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.5,markersize=1)

# ax.plot(test_virtual_origin_poly_x,test_virtual_origin_poly_y,color='red',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1,markersize=1)
# ax.plot(test_move_pts_x,test_move_pts_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1,markersize=1)
# ax.plot(test_111_poly_x,test_111_poly_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1,markersize=1)
# ax.plot(cgal_pts_entrance_x,cgal_pts_entrance_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 1,markersize=1)

# ax.plot(test_x,test_y,color='g',markerfacecolor='green',marker='o',label='keypoints data')
# ax.plot(NN_x,NN_y,color='g',markerfacecolor='green',marker='o',label='keypoints data')
#
# ax.plot(DD_x,DD_y,color='r',markerfacecolor='green',marker='o',label='realline data')

# for a, b in zip(CC_x,CC_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(entrance_lines_x,entrance_lines_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# #
# #
# for a, b in zip(testAB_x,testAB_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(testABtransd_x,testABtransd_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(test_virtual_origin_poly_x,test_virtual_origin_poly_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

# for a, b in zip(test_skeleton_4_x, test_skeleton_4_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(test_txt_x, test_txt_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

# for a, b in zip(test1_x,test1_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

# # 绘制线段
for i in range(0, len(lineshow[0])-1, 2):
    x = [lineshow[0][i], lineshow[0][i+1]]
    y = [lineshow[1][i], lineshow[1][i+1]]
    plt.plot(x, y)
    plt.text(lineshow[0][i], lineshow[1][i], f"({lineshow[0][i]}, {lineshow[1][i]})")

ax.set_xlabel('x label')
ax.set_ylabel('y label')
ax.set_title('Simple Plot')
ax.set_aspect('equal')
# ax.autoscale()
ax.legend()

# plt.xlim(0, 700) # 横坐标显示范围为0到6
# plt.ylim(0, 1500)

# mng = plt.get_current_fig_manager()
# mng.full_screen_toggle()
fig.tight_layout()

plt.show()



