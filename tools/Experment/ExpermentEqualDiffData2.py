data1 = [34.3113, 45.3033, 45.3033, 66.6762, 66.6762, 97.8528, 97.8528, 98.5258]
data2 = [29.0113, 75.8013, 75.8013, 90.4266, 90.4266, 116.393, 116.393, 116.869]

for i in range(len(data1) - 2):
    # 获取第一行的3个数据
    x1, x2, x3 = data1[i:i+3]

    # 获取第二行的3个数据
    y1, y2, y3 = data2[i:i+3]

    # 在这里进行对应的操作，例如调用PJcurvature函数
    # 注意：需要将x1, x2, x3, y1, y2, y3作为参数传递给PJcurvature函数

    # 示例：打印获取到的数据
    print("Data set", i // 3 + 1)
    print("x1:", x1)
    print("x2:", x2)
    print("x3:", x3)
    print("y1:", y1)
    print("y2:", y2)
    print("y3:", y3)
    print()