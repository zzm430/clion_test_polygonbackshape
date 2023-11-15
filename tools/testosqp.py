import osqp
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
import random

# 道路上放置障碍物，共设置4个障碍物
# 每个障碍物obj 使用4个数值来表示，障碍物形状为矩形，四个数据依次表示 start_s,end_s,l_low,l_up
obs = [[5, 10, -1, 1.5], [18, 22, -3, -2], [25, 30, 0, 1]]

s_len = 50
delta_s = 0.1
# n = 500
n = int(s_len / delta_s)
# len(x) = 500
x = np.linspace(0, s_len, n)
# len(bound) = 2503
up_bound = [0] * (5 * n + 3)
low_bound = [0] * (5 * n + 3)
# len(s_ref) = 1500
s_ref = [0] * 3 * n

dddl_bound = 0.01

####################边界提取################
l_bound = 2
safe_delta_l = 0.1
for i in range(n):
    for j in range(len(obs)):
        if (x[i] >= obs[j][0] and x[i] <= obs[j][1]) and ((obs[j][2] < l_bound) and (obs[j][3] > - l_bound)):
            if (obs[j][2] + (obs[j][3] - obs[j][2]) / 2) >= 0:
                low_ = -l_bound
                up_ = obs[j][2] - safe_delta_l
            else:
                low_ = obs[j][3] + safe_delta_l
                up_ = l_bound
            break
        else:
            up_ = l_bound
            low_ = -l_bound
    # 先根据障碍物调整边界
    up_bound[i] = up_
    low_bound[i] = low_
    # reference line 为上下边界的中线
    s_ref[i] = 0.5 * (up_ + low_)

####################构造P和Q################
w_l = 50000
w_dl = 1000
w_ddl = 1
w_dddl = 1
eye_n = np.identity(n)
zero_n = np.zeros((n, n))

P_zeros = zero_n
P_l = w_l * eye_n
P_dl = w_dl * eye_n
P_ddl = (w_ddl + 2 * w_dddl / delta_s / delta_s) * eye_n - 2 * w_dddl / delta_s / delta_s * np.eye(n, k=-1)
P_ddl[0][0] = w_ddl + w_dddl / delta_s / delta_s
P_ddl[n - 1][n - 1] = w_ddl + w_dddl / delta_s / delta_s

# p.shape = 1500 * 1500
P = sparse.csc_matrix(np.block([
    [P_l, P_zeros, P_zeros],
    [P_zeros, P_dl, P_zeros],
    [P_zeros, P_zeros, P_ddl]
]))
# q.shape = 1500
q = np.array([-w_l * s_ for s_ in s_ref])

####################构造A和LU################

# 构造：l(i+1) = l(i) + l'(i) * delta_s + 1/2 * l''(i) * delta_s^2 + 1/6 * l'''(i) * delta_s^3
A_ll = -eye_n + np.eye(n, k=1)
A_ldl = -delta_s * eye_n
A_lddl = -0.5 * delta_s * delta_s * eye_n
A_l = (np.block([
    [A_ll, A_ldl, A_lddl]
]))

# 构造：l'(i+1) = l'(i) + l''(i) * delta_s + 1/2 * l'''(i) * delta_s^2
A_dll = zero_n
A_dldl = -eye_n + np.eye(n, k=1)
A_dlddl = -delta_s * eye_n
A_dl = np.block([
    [A_dll, A_dldl, A_dlddl]
])

A_ul = np.block([
    [eye_n, zero_n, zero_n],
    [zero_n, zero_n, zero_n],
    [zero_n, zero_n, zero_n]
])  # 3n*3n
# 初始化设置
A_init = np.zeros((3, 3 * n))
A_init[0][0] = 1

# A.shape = 2503 * 1500
A = sparse.csc_matrix(np.row_stack((A_ul, A_l, A_dl, A_init)))

low_bound[5 * n] = 1
up_bound[5 * n] = 1
# l.shape = 2503 * 1
l = np.array(low_bound)
# u.shape = 2503 * 1
u = np.array(up_bound)

# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace and change alpha parameter
prob.setup(P, q, A, l, u, alpha=1.0)

# Solve problem
res = prob.solve()

s_seg = range(0, 500, 1)
fig = plt.figure()
ax = fig.add_subplot(111)

# obs = [[5,10,1,2],[15,20,-2,-0.5],[35,39,0,1.5]]
# rect1 = plt.Rectangle((50,1),100,2)
# ax.add_patch(rect1)
# rect2 = plt.Rectangle((150,-2),200,-0.5)
# ax.add_patch(rect2)

ax.plot(s_seg, u[:n], '.', color='blue')
ax.plot(s_seg, l[:n], '.', color='black')
ax.plot(s_seg, s_ref[:n], '.', color='yellow')
ax.plot(s_seg, res.x[:n], '.', color='red')

plt.show()