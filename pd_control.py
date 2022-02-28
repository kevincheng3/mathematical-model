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

x0 = np.array([-0.35, 1.3, 0.6,0.3])
res = minimize(cost, x0)  
        #    constraints=[linear_constraint, nonlinear_constraint],


def torque_computed(current_pos, current_vel, desired_pos, desired_vel, kp=20, kv=0.01 ):
    pos_torque = - kp * (current_pos - desired_pos)
    vel_torque = - kv * (current_vel - desired_vel)
    # print(pos_torque, vel_torque)
    return pos_torque + vel_torque

def jacobian(id, theta0, theta1, theta2, theta3):
    l = 0.03
    dot_x = np.array([-(l * cos(theta1) + l * cos(theta1+theta2) + l * cos(theta1 + theta2 + theta3)) * sin(theta0), 
            -l * cos(theta0) * sin(theta1), -l * cos(theta0) * sin(theta1 +theta2), -l * cos(theta0) * sin(theta1 +theta2 + theta3)])
    dot_y= np.array([(l * cos(theta1) + l * cos(theta1+theta2) + l * cos(theta1 + theta2 + theta3)) * cos(theta0), 
            -l * sin(theta0) * sin(theta1), -l * sin(theta0) * sin(theta1 +theta2), -l * sin(theta0) * sin(theta1 +theta2 + theta3)])
    dot_z= np.array([0 , l * cos(theta1), l * cos(theta1 +theta2), l * cos(theta1 +theta2 + theta3)])
    if id ==0:
        return np.array([dot_x, dot_y, dot_z])
    elif id == 1:
        return np.array([-dot_y, dot_x, dot_z])
    elif id == 2:
        return np.array([-dot_x, -dot_y, dot_z])
    elif id == 3:
        return np.array([dot_y, -dot_x, dot_z])

class Controller():
    # The max speed.
    MAX_SPEED = 1.0

    # The minimum speed.
    MIN_SPEED = 0.0
    SPEED_CHANGE_PERCENT = 0.2

    def __init__(self, sim) -> None:
        super().__init__()
        self.sim = sim

    def apply_torque(self, id, desired_pos, desired_vel, kp=20, kv=0.01 ):
        pos_torque = - kp * (self.sim.data.qpos[id+3] - desired_pos)
        vel_torque = - kv * (self.sim.data.qvel[id+3] - desired_vel)

        self.sim.data.ctrl[id] = pos_torque + vel_torque
        # print(pos_torque, vel_torque)
        # return pos_torque + vel_torque

    def apply_torque_jacobian(self, id, desired_pos, desired_vel, kp=20, kv=0.01 ):
        pos_torque = - kp * ( - desired_pos)
        vel_torque = - kv * (self.sim.data.qvel[id+3] - desired_vel)

        self.sim.data.ctrl[id] = pos_torque + vel_torque

    

def main():
    model = load_model_from_path("./planar_box_pd.xml")
    sim = MjSim(model)
    hand = Controller(sim)
    viewer = MjViewer(sim)    

    viewer.cam.distance = 0.5
    viewer.cam.azimuth = 0
    viewer.cam.elevation = -20
    viewer._paused = 1

    x0 = np.array([-0.35, 1.3, 0.6,0.3])
    res = minimize(cost, x0)  

    delta_t = 0.0002
    n_inters = 999
    n_fingers = 4


    for i in range (n_inters+1):
        for j in range(n_fingers):
            hand.apply_torque(1 + 4*j, res.x[1] /  n_inters * i, 0, kp=10, kv = 0.01)
            hand.apply_torque(2 + 4*j, res.x[2] /  n_inters * i, 0)
            hand.apply_torque(3 + 4*j, res.x[3] /  n_inters * i, 0)            
            if j % 2 ==0:
                hand.apply_torque(0 + 4*j, -res.x[0] / n_inters * i, 0, kp=10, kv = 0.01)
            else:
                hand.apply_torque(0 + 4*j, res.x[0] / n_inters * i, 0, kp=10, kv = 0.01)

        sim.step()
        viewer.render()

    for i in range(1000):
        for j in range(n_fingers):
            hand.apply_torque(1 + 4 * j, res.x[1], 0, kp=10, kv = 0.01)
            hand.apply_torque(2 + 4 * j, res.x[2], 0)
            hand.apply_torque(3 + 4 * j, res.x[3], 0)            
            if j % 2 ==0:
                hand.apply_torque(0 + 4 * j, -res.x[0], 0, kp=10, kv = 0.01)
            else:
                hand.apply_torque(0 + 4 * j, res.x[0], 0, kp=10, kv = 0.01)

        sim.step()
        viewer.render()

    # while 1:
    for i in range(5000):
        v = np.array([0.0, -0.008, 0.0])
        theta = np.zeros((n_fingers, 4))
        for j in range(n_fingers):
            theta = np.linalg.pinv(jacobian(j, sim.data.qpos[3 + 4 * j], sim.data.qpos[4 + 4 * j], sim.data.qpos[5  + 4 * j], sim.data.qpos[6  + 4 * j])).dot(v)
            for k in range(4):
                if k == 0:
                    hand.apply_torque_jacobian(k + 4 * j, theta[k] * delta_t, theta[k], kp=10, kv = 0.01)
                else: 
                    hand.apply_torque_jacobian(k + 4 * j, theta[k] * delta_t, theta[k])

        sim.step()
        viewer.render()

    for i in range(100000):
        v = np.array([0.008 * cos(i / (1000 * np.pi)) , 0.008 * sin(i / (1000 * np.pi)), 0.0])
        # print(v)
        theta = np.zeros((n_fingers, 4))
        for j in range(n_fingers):
            theta = np.linalg.pinv(jacobian(j, sim.data.qpos[3 + 4 * j], sim.data.qpos[4 + 4 * j], sim.data.qpos[5  + 4 * j], sim.data.qpos[6  + 4 * j])).dot(v)
            for k in range(4):
                if k == 0:
                    hand.apply_torque_jacobian(k + 4 * j, theta[k] * delta_t, theta[k], kp=10, kv = 0.01)
                else: 
                    hand.apply_torque_jacobian(k + 4 * j, theta[k] * delta_t, theta[k])


        sim.step()
        viewer.render()



    # while 1:
        # for j in range(n_fingers):
        #     hand.apply_torque(1 + 4 * j, res.x[1], 0, kp=20, kv = 0.01)
        #     hand.apply_torque(2 + 4 * j, res.x[2], 0)
        #     hand.apply_torque(3 + 4 * j, res.x[3], 0)            
        #     if j % 2 ==0:
        #         hand.apply_torque(0 + 4 * j, -res.x[0], 0, kp=10, kv = 0.01)
        #     else:
        #         hand.apply_torque(0 + 4 * j, res.x[0], 0, kp=10, kv = 0.01)
        # sim.data.ctrl[-1] = 1.2
        # sim.data.ctrl[-2] = 13
        # sim.data.ctrl[-3] = 11
        # print(sim.data.sensordata)

if __name__ == "__main__":
    main()