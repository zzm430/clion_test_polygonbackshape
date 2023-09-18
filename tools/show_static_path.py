import matplotlib.pyplot as plt
import numpy as np
import math

# a=np.loadtxt("/home/zzm/Downloads/reeds_shepp-master/RS_Lib/1.txt")  #首先使用Numpy的loadtxt函数从文件中读取路径数据并存储到变量a中
a=np.loadtxt("/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt")

# a=np.loadtxt("/home/zzm/Desktop/test_path_figure-main/src/routing_ps.txt")
# B = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/origin_polygon.txt')
# c = np.loadtxt('/home/zzm/Desktop/test_path_figure-main/src/testtemp.txt')
# c_x = c[0]
# c_y = c[1]
# B_x = B[0]
# B_y = B[1]

f=plt.figure();                                                      #创建一个Figure对象，并向其中添加一个包含子图的Axes对象
ax=f.add_subplot(111)                                                
# ax.set(xlim=[523300, 523900],ylim=[4897000,4897900 ])                                   #设置x轴和y轴的范围限制为[0,10]
# ax.set(xlim=[0,500],ylim=[0,500])
# ax.set(xlim=[-400, 500], ylim=[-400,500 ])
# ax.plot(B_x,B_y,color='r',markerfacecolor='red',marker='o',label='update data')
# ax.plot(c_x,c_y,color='y',markerfacecolor='red',marker='o',label='update1 data')
plt.plot(a[0,0],a[0,1],'ro')


#使用plt.plot函数绘制起点和终点，并使用plt.arrow函数在其上添加一条箭头表示前进方向
# plt.arrow(a[0,0], a[0,1], 1.0 * math.cos(a[0,2]), 1.0 * math.sin(a[0,2]),
#                   fc="r", ec="k", head_width=0.5, head_length=0.5)
plt.plot(a[len(a)-1,0],a[len(a)-1,1],'yo')
# plt.arrow(a[len(a)-1,0], a[len(a)-1,1], 1.0 * math.cos(a[len(a)-1,2]), 1.0 * math.sin(a[len(a)-1,2]),fc="r", ec="k", head_width=0.5, head_length=0.5)
plt.plot(a[:,0],a[:,1],'-b',linewidth=0.3)                                           #绘制整个路径，将结果显示出来
# plt.plot(a[:,0],a[:,1],'ro',markersize= 8)
# for i in range(len(a)):
#     plt.plot(a[i,0],a[i,1],'ro', markersize=8)                         # 绘制每个点
#     plt.text(a[i,0]+0.2,a[i,1]+0.2,f"({a[i,0]}, {a[i,1]})")                      # 在每个点旁边添加坐标的文本说明
#
# m = [-2.3,-1.5, -2.3 ,  -1.5 ,  -2.3 ,-1.5 ]
# n = [-2 , -2,  -2 , -2, 2  ,2]
# plt.plot(m, n, 'ro')  # 'ro'表示红色的圆点
#
# for i in range(len(m)):
#     ax.annotate(f"({m[i]}, {m[i]})", (m[i], m[i]))

plt.axis('equal')  # 让横坐标间隔等于纵坐标间隔
plt.show()
