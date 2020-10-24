import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
# Differences between cubic spline and splprep
# ----------------------------------------------------------------------------------------------------------------------
"""
points = [[0,1], [1, 1.1],[2, 1.2],[3,3],[4,2.9],[5,2.8],[6,2.7]]

x = [point[0] for point in points]
y = [point[1] for point in points]

f = np.polyfit(x, y, deg=6)
p = np.poly1d(f)

ls = interpolate.interp1d(x, y, kind = 'linear')
qs = interpolate.interp1d(x, y, kind = 'quadratic')
cs = interpolate.interp1d(x, y, kind = 'cubic')

x_new = np.linspace(x[0], x[-1], 100)

plt.scatter(x, y, label='data')
plt.plot(x_new, p(x_new), label = 'polynomial p6')
plt.plot(x_new, ls(x_new), label = 'linear spline')
plt.plot(x_new, qs(x_new), label = 'quadratic spline')
plt.plot(x_new, cs(x_new), label = 'cubic spline')
plt.legend(loc = 'upper left')

tck,u = interpolate.splprep([x,y], k = 3, s = 0)
spline = interpolate.splev(np.linspace(0,1,100),tck)

plt.scatter(x, y, label='data')
plt.plot(x_new, cs(x_new), label = 'cubic spline')
#plt.plot(x_new, f(x_new), label = 'Akima Interpolator')
plt.plot(spline[0], spline[1], label = "splprep")
plt.legend(loc = 'upper left')
plt.show()
"""
step= np.linspace(0.0,1.0,15)
print(step)