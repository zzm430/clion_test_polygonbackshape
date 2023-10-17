import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import mplcursors

# 生成示例数据
x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)
X, Y = np.meshgrid(x, y)
Z = X**2 + Y**2

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z)

# 添加mplcursors事件
cursor = mplcursors.cursor(ax)
cursor.connect("add", lambda sel: sel.annotation.set_text(f"({sel.target[0]:.2f}, {sel.target[1]:.2f}, {sel.target[2]:.2f})"))

plt.show()