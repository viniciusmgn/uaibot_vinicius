import uaibot as ub
import numpy as np


points = np.array([[1,2,3],[4,5,6],[7,8,9]])
print(points)
f = ub.Utils.interpolate(points)

print(f(0)) #primeiro ponto, [1,4,7]
print(f(1/3)) #segundo ponto, [2,5,8]
print(f(2/3)) #terceiro ponto, [3,6,9]
print(f(1)) #primeiro ponto novamente, pois a função tem período 1
print(f(0.2)) #ponto intermediário entre o primeiro e o segundo
