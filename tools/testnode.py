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

test_txt = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/test.txt')
test_txt_x = test_txt[0]
test_txt_y = test_txt[1]


entrance_lines = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/cgal_pts_entrance.txt')
entrance_lines_x = entrance_lines[0]
entrance_lines_y = entrance_lines[1]

convexHull = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/convexHull.txt')
convexHull_x = convexHull[0]
convexHull_y = convexHull[1]

# cgal_show_ridge_path = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt')
# cgal_show_ridge_path_x = cgal_show_ridge_path[0]
# cgal_show_ridge_path_y = cgal_show_ridge_path[1]
# ax.plot(ff_x,ff_y,color='b',markerfacecolor='green',marker='o',label='narrow_points data')
# ax.plot(EE_x,EE_y,color='b',markerfacecolor='green',marker='o',label='increaseNodes data')
# ax.plot(EE_x,EE_y,color='b',markerfacecolor='green',marker='o',label='increaseNodes data')
# ax.plot(DD_x,DD_y,color='y',markerfacecolor='green',marker='o',label='inter_nodes data')
# ax.plot(BB_x,BB_y,color='r',markerfacecolor='green',marker='o',label='origin_polygon data')
ax.plot(CC_x,CC_y,color='g',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)
# ax.plot(convexHull_x,convexHull_y,color='blue',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 0.3,markersize=1)

# ax.plot(test_txt_x,test_txt_y,color='black',markerfacecolor='green',marker='o',label='keypoints data',linewidth= 3.3,markersize=1)
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
# for a, b in zip(BB_x,BB_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(test_virtual_origin_poly_x,test_virtual_origin_poly_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)

# for a, b in zip(test_skeleton_4_x, test_skeleton_4_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# for a, b in zip(ff_x, ff_y):
#         plt.text(a, b, (a, b), ha='center', va='bottom', fontsize=10)
# 绘制线段
# for i in range(0, len(test_skeleton_2[0])-1, 2):
#     x = [test_skeleton_2[0][i], test_skeleton_2[0][i+1]]
#     y = [test_skeleton_2[1][i], test_skeleton_2[1][i+1]]
#     plt.plot(x, y)
#     plt.text(test_skeleton_2[0][i], test_skeleton_2[1][i], f"({test_skeleton_2[0][i]}, {test_skeleton_2[1][i]})")

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