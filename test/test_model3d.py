import uaibot as ub
import numpy as np


points = np.array([[1,2,3],[2,0,0],[3,-1,1]])
print(points)
f = ub.Utils.interpolate(points)

print(f(0.1))