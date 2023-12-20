import matplotlib.pyplot as plt

# 读取txt文件中的数据
with open('/home/zzm/Desktop/test_path_figure-main/src/test_pjpo_curve.txt', 'r') as file:
    lines = file.readlines()

# 计算数据集数量，以确定子图数量
num_datasets = len(lines)

# 创建子图
fig, axs = plt.subplots(num_datasets, 1, figsize=(8, 6), sharex=True, sharey=True)

# 遍历每一行数据，为每个子图绘制折线图
for i, line in enumerate(lines):
    # 将数据转换为数值类型
    data = [float(x) for x in line.strip().split()]

    # 绘制折线图
    axs[i].plot(data, color='g', markerfacecolor='green', marker='o', label='keypoints data')

    # 显示每个数据点的标签
    for j, d in enumerate(data):
        axs[i].text(j, d, str(d), ha='center', va='bottom', fontsize=10)

    # 添加标题和坐标轴标签
    axs[i].set_title('Data Line Chart')
    axs[i].set_xlabel('Index')
    axs[i].set_ylabel('Value')

# 调整子图间距和整个图形的布局
plt.tight_layout()

# 显示图形
plt.show()