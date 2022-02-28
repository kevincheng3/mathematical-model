import numpy as np
from mujoco_py import MjSim, MjViewer, load_model_from_path

import numpy as np
from scipy.spatial.transform import Rotation as R
from math import cos, sin, atan
from scipy.optimize import minimize

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

def cost(x):
    desired_pos = np.array([0.02, -0.02, 0.07])
    pos = forward(x[0], x[1], x[2], x[3])
    cost = np.linalg.norm(desired_pos - pos)
    # print(pos)
    return cost

# x0 = np.array([-0.35, 1.3, 0.6,0.3])
# res = minimize(cost, x0)  
#         #    constraints=[linear_constraint, nonlinear_constraint],
# print(res.x)


def jacobian(theta0, theta1, theta2, theta3):
    l = 0.03
    dot_x = np.array([-(l * cos(theta1) + l * cos(theta1+theta2) + l * cos(theta1 + theta2 + theta3)) * sin(theta0), 
            -l * cos(theta0) * sin(theta1), -l * cos(theta0) * sin(theta1 +theta2), -l * cos(theta0) * sin(theta1 +theta2 + theta3)])
    dot_y= np.array([(l * cos(theta1) + l * cos(theta1+theta2) + l * cos(theta1 + theta2 + theta3)) * cos(theta0), 
            -l * sin(theta0) * sin(theta1), -l * sin(theta0) * sin(theta1 +theta2), -l * sin(theta0) * sin(theta1 +theta2 + theta3)])
    dot_z= np.array([0 , l * cos(theta1), l * cos(theta1 +theta2), l * cos(theta1 +theta2 + theta3)])
    # print(np.array([dot_x, dot_y, dot_z]))
    return np.array([dot_x, dot_y, dot_z])
    

def main():
    model = load_model_from_path("./planar_box.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)    
    # body_id = sim.model.body_name2id('box')
    # lookat = sim.data.body_xpos[body_id]
    # for idx, value in enumerate(lookat):
    #     viewer.cam.lookat[idx] = value
    viewer.cam.distance = 0.5
    viewer.cam.azimuth = 0
    viewer.cam.elevation = -20
    viewer._paused = 1

    x0 = np.array([-0.35, 1.3, 0.6,0.3])
    res = minimize(cost, x0)  
            #    constraints=[linear_constraint, nonlinear_constraint],
    # print(res.x)
    for i in range (2000):
        sim.data.qpos[3:7] = [0, 1, 1, 1]
    # # #     sim.data.ctrl[0] = -res.x[0]
    # # #     sim.data.ctrl[8] = -res.x[0]
    # # #     sim.data.ctrl[1:4] = res.x[1:4]
    # # #     sim.data.ctrl[4:8] = res.x
    # # #     sim.data.ctrl[9:12] = res.x[1:4]
    # # #     sim.data.ctrl[12:16] = res.x
    # # #     print("res", res.x)

    # # #     print("pos", sim.data.qpos[3:7])
    # #     print(sim.data.qpos[3:7])
    #     sim.step()
    #     viewer.render()
    while 1:
        v = np.array([0.01, 0.0, 0.0])
        theta = np.linalg.pinv(jacobian(sim.data.qpos[3], sim.data.qpos[4], sim.data.qpos[5], sim.data.qpos[6])).dot(v)
        print(jacobian(sim.data.qpos[3], sim.data.qpos[4], sim.data.qpos[5], sim.data.qpos[6]))
        # theta = np.linalg.lstsq((jacobian(sim.data.qpos[3], sim.data.qpos[4], sim.data.qpos[5], sim.data.qpos[6])), v, rcond = -1)
        print("theta", theta)
        # print(theta)
        # sim.data.ctrl[0:4] = sim.data.qpos[3:7].copy() + theta[0] * 0.0002
        sim.data.ctrl[0:4] = sim.data.qpos[3:7].copy() + theta * 0.0002
        # sim.data.qpos[3:7] = sim.data.qpos[3:7].copy() + theta[0].copy() * 0.0002
        print("qpos", sim.data.qpos[3:7])
        # print(sim.data.ctrl[0:4])
        # print(sim.data.site_xpos)
        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()