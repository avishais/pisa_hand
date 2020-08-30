import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import glob

D = glob.glob("../meshes/*.stl")

for d in D:    
    if d.find('_t.') > -1 or d.find('01_chips_can') < 0:
        continue

    figure = plt.figure()
    axes = mplot3d.Axes3D(figure)

    ed = d.find('.stl')
    S = d[10:ed]
    print("File: " + S)

    # Load the STL files and add the vectors to the plot
    obj = mesh.Mesh.from_file('../meshes/' + S + '.stl')

    P = obj.points.reshape(-1,3)

    mx = np.max(P, axis=0)
    mn = np.min(P, axis=0)
    c = (mx + mn) / 2

    # print(obj.vectors)
    obj.vectors -= c

    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(obj.vectors))

    # Auto scale to the mesh size
    scale = obj.points.flatten(-1)

    axes.auto_scale_xyz(scale, scale, scale)

    obj.save('../meshes/' + S + '_t.stl')

    # Show the plot to the screen
    plt.show()