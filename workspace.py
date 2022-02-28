import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from math import cos, sin, atan
from matplotlib import cm
def rotation(theta_x=0, theta_y=0, theta_z=0):

    rot_x = np.array([[1, 0, 0],[0, np.cos(theta_x), - np.sin(theta_x)], [0, np.sin(theta_x), np.cos(theta_x)]])
    rot_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],[0, 1, 0], [-np.sin(theta_y), 0, np.cos(theta_y)]])
    rot_z = np.array([[np.cos(theta_z), - np.sin(theta_z), 0],[ np.sin(theta_z), np.cos(theta_z), 0], [0, 0, 1]])
    R = rot_x.dot(rot_y).dot(rot_z)

    return R

def quat2euler(quat):
    # transfer quat to euler
    r = R.from_quat(np.array([quat[1], quat[2], quat[3], quat[0]]))
    return r.as_euler('XYZ')

def forward(theta0, theta1, theta2, theta3):
    l = 0.03
    r0 = R.from_euler('z', theta0)
    r1 = R.from_euler('x', theta1)
    r2 = R.from_euler('x', theta2)
    r3 = R.from_euler('x', theta3)

    R1 = r0 * r1
    R2 = R1 * r2 
    R3 = R2 * r3

    v0 = np.array([0, l, 0])
    v1 = [0, l, 0]
    v2 = [0, l, 0]

    pos1 =  R1.apply(v0)
    pos2 =  R2.apply(v1)
    pos3 =  R3.apply(v2)
    # pos = pos1 + pos2 + pos3
    pos = pos1 + pos2 + pos3
    # print((self.R0))
    return pos
data=[]
for i in range(11):
    theta0 = np.pi/20 *i  - np.pi / 4
    for j in range(11):
        theta1 = np.pi/20 *j
        for k in range(11):
            theta2 = np.pi/20 *k
            for m in range(11):
                theta3 = np.pi/20 *m
                a = forward(theta0, theta1, theta2, theta3)
                data.append(a)
data = np.array(data)
# print(data[0])
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# for i in range(data.shape[0]):
#     ax.scatter(data[i][0], data[i][1],data[i][2],  marker='o')

ax.scatter(data[:,0], data[:,1], data[:,2], color="gray", marker='o', alpha= 0.2)
x = np.linspace(-0.02, 0.02, 10)
y = np.linspace(-0.04, 0, 10)
X, Y = np.meshgrid(x, y)
# ax.plot_surface(X=X,Y=Y,Z=X*0 + 0.07, color='r',alpha=0.6) 
ax.plot_surface(X=X,Y=Y-0.02,Z=X*0 + 0.085, color='c',alpha=0.6) 
ax.plot_surface(X=X,Y=Y-0.02,Z=X*0 + 0.055, color='c',alpha=0.6) 
ax.plot_surface(X=X*0 + 0.02,Y=Y-0.02,Z=X*0.75 + 0.07, color='c',alpha=0.6) 
ax.plot_surface(X=X*0 - 0.02,Y=Y-0.02,Z=X*0.75 + 0.07, color='c',alpha=0.6) 
ax.plot_surface(X=Y+0.02, Y=X*0-0.02,Z=X*0.75 + 0.07, color='c',alpha=0.6) 
ax.plot_surface(X=Y+0.02, Y=X*0-0.06,Z=X*0.75 + 0.07, color='c',alpha=0.6) 

# cube1 = ( -0.02< x < 0.02) & ( -0.06 <y < -0.02) & (0.055 < z < 0.085)
# voxelarray = cube1
# ax.voxels(voxelarray, edgecolor='k')
# point = np.array([0.02, 0, 0.07])

# ax.scatter( 0.02, 0, 0.07, color = 'r', marker='o')

# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

plt.show()