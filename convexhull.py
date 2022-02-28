import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

# 8 points defining the cube corners
pts = np.array([[-1, 0, 1], [0, -1, -1], [1, 0, 1], [0, 1, -1],
                 ])

# pts2 = np.array([[-1, 0, -0.5], [0, -1, 0.5], [1, 0, -0.5], [0, 1, 0.5],
#                  ])


hull = ConvexHull(pts)
# hull2 = ConvexHull(pts2)


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot defining corner points
ax.plot(pts.T[0], pts.T[1], pts.T[2], "ko")
ax.scatter(0, 0, 0, color="black", marker="^")

# ax.plot(pts2.T[0], pts2.T[1], pts2.T[2], "ko")

# 12 = 2 * 6 faces are the simplices (2 simplices per square face)

for s in hull.simplices:
    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
    ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "c-")

# for s in hull2.simplices:
#     s = np.append(s, s[0])  # Here we cycle back to the first coordinate
#     ax.plot(pts2[s, 0], pts2[s, 1], pts2[s, 2], "g-")

x, y, z = np.zeros((3,3))
u, v, w = np.array([[2,0,0],[0,2,0],[0,0,2]])

ax.quiver(x,y,z,u,v,w,arrow_length_ratio=0.1, colors="black")

# Make axis label
for i in ["x", "y", "z"]:
    eval("ax.set_{:s}label('{:s}')".format(i, i))

plt.show()

import numpy as np

def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0
# pts = np.array([[-1, 0, -0.1], [0, -1, 0.1], [1, 0,- 0.2], [0, 1, 0.2]
#                  ])
print(in_hull(np.array([0, 0, 0]), pts))

# print(hull.volume)
# print(hull2.volume)