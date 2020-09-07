import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

D = np.loadtxt("/home/avishai/catkin_ws/src/pisa_hand/object_spawn/data/finger_logs_o13_f5.txt")

X = []
for d in D:
    for x in d.reshape(5,3):
        X.append(x)

X = np.array(X)

figure = plt.figure()
ax = plt.axes(projection='3d')

ax.scatter3D(X[:,0], X[:,1], X[:,2], 'gray')
plt.show()
