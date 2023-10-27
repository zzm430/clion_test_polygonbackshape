data1 = [34.3113, 45.3033, 45.3033, 66.6762, 66.6762, 97.8528, 97.8528, 98.5258]
data2 = [29.0113, 75.8013, 75.8013, 90.4266, 90.4266, 116.393, 116.393, 116.869]

# 使用迭代器和zip函数同时遍历两行数据
iterator = iter(zip(data1, data2))

for i in range(len(data1) // 3):
    # 获取下一个3个数据
    x1, y1 = next(iterator)
    x2, y2 = next(iterator)
    x3, y3 = next(iterator)

    # 在这里进行对应的操作，例如调用PJcurvature函数
    # 注意：需要将x1, y1, x2, y2, x3, y3作为参数传递给PJcurvature函数

    # 示例：打印获取到的数据
    print("Data set", i+1)
    print("x1:", x1)
    print("y1:", y1)
    print("x2:", x2)
    print("y2:", y2)
    print("x3:", x3)
    print("y3:", y3)
    print()