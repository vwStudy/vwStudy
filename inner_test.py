import numpy as np
import math

a = np.array([-2, -0.5])
b = np.array([-2, 0.5])

#a = np.array([0.7, 2.9])
#b = np.array([-0.2,  3.0])


dot = np.dot(a ,b)
#inner = float(inner)
print("dot", dot)

an = np.linalg.norm(a)
bn = np.linalg.norm(b)

an = round(an, 5)
bn = round(bn ,5)
print("an",an)
print("bn", bn)


print("test2", dot/(an * bn))
theta = math.acos(dot/(an * bn))
print("test1", theta)
degree = math.degrees(theta)
print("car3",degree)