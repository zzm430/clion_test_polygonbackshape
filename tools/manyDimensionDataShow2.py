import matplotlib.pyplot as plt

# 读取txt文件中的数据
with open('/home/zzm/Desktop/test_path_figure-main/src/test_pjpo_curve.txt', 'r') as file:
    lines = file.readlines()

# 计算数据集数量，以确定子图数量
num_datasets = len(lines)

# 计算每一行子图的数量
num_subplots_per_row = 6
num_rows = (num_datasets + num_subplots_per_row - 1) // num_subplots_per_row

# 创建子图
fig, axs = plt.subplots(num_rows, num_subplots_per_row, figsize=(18, 8), sharex=True, sharey=True)

# 遍历每一行数据，为每个子图绘制折线图
for i, line in enumerate(lines):
    # 将数据转换为数值类型
    data = [float(x) for x in line.strip().split()]

    # 确定当前子图的位置
    row_idx = i // num_subplots_per_row
    col_idx = i % num_subplots_per_row

    # 绘制折线图
    axs[row_idx, col_idx].plot(data, color='g', markerfacecolor='green', marker='o',markersize=2, label='keypoints data')

    # # 显示每个数据点的标签
    # for j, d in enumerate(data):
    #     axs[row_idx, col_idx].text(j, d, str(d), ha='center', va='bottom', fontsize=10)

    # 添加标题和坐标轴标签
    axs[row_idx, col_idx].set_title('Data Line Chart')
    axs[row_idx, col_idx].set_xlabel('Index')
    axs[row_idx, col_idx].set_ylabel('Value')

    axs[row_idx, col_idx].set_ylim(-0.3, 0.3)

# 调整子图间距和整个图形的布局
plt.tight_layout()

# 显示图形
plt.show()
