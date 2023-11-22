import numpy as np
from math import *
import matplotlib.pyplot as plt

theta = np.linspace(0, np.pi, 10)
x = 5.0 * np.cos(theta)
y = 5.0 * np.sin(theta)
s = theta * 5.0

fx = np.poly1d(np.polyfit(s, x, 5))
dfx = fx.deriv()

fy = np.poly1d(np.polyfit(s, y, 5))
dfy = fy.deriv()

snew = np.linspace(0, 5 * np.pi, 1000)

newx = fx(snew)
newy = fy(snew)

plt.plot(np.zeros(1000), snew, 'y')
plt.plot(np.zeros(1000) + 1.5, snew, 'b')
plt.plot(np.zeros(1000) - 1.5, snew, 'b')


def polyfit(coeffs, t):
    return coeffs[0] + coeffs[1] * t + coeffs[2] * t * t + coeffs[3] * t * t * t


def cubicPolyCurve1d(x0, dx0, x1, dx1, T):
    coeffs = np.zeros(4)
    coeffs[0] = x0
    coeffs[1] = dx0

    T2 = T * T

    coeffs[2] = (3 * x1 - T * dx1 - 3 * coeffs[0] - 2 * coeffs[1] * T) / T2
    coeffs[3] = (dx1 - coeffs[1] - 2 * coeffs[2] * T) / (3.0 * T2)
    return coeffs


targets = []
for i in range(11):
    x0 = 0
    dx0 = 0  # np.tan(np.pi/6)
    x1 = -1.5 + i * 0.3
    dx1 = 0
    targets.append(np.array([x0, dx0, x1, dx1]))

t = np.linspace(0, 10, 1000)
for i in range(11):
    tar = targets[i]
    coeffs = cubicPolyCurve1d(tar[0], tar[1], tar[2], tar[3], 10.0)
    d = polyfit(coeffs, t)
    plt.plot(d, t, 'g')
plt.axis([-8, 8, 0, 16])
plt.show()
