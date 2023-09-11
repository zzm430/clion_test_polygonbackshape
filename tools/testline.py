import matplotlib.pyplot as plt

# 从txt文件读取坐标点
with open("coordinates.txt", "r") as f:
    lines = f.readlines()

points = []
for line in lines:
    x, y = line.strip().split(" ")
    points.append((float(x), float(y)))

# 将坐标点分成线段
segments = [(points[i], points[i+1]) for i in range(0, len(points)-1, 2)]

# 绘制线段
for segment in segments:
    x_values = [segment[0][0], segment[1][0]]
    y_values = [segment[0][1], segment[1][1]]
    plt.plot(x_values, y_values)

plt.show()