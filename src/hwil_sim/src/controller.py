#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import numpy as np
import control
from states import States
from pubsub import PubSub
import csv
import matplotlib.pyplot as plt

m = 1.477
g = 9.8065
Ixx = 0.014641566666666668
Iyy = 0.014641566666666668
Izz = 0.026640733333333336
l = 0.275
k = 0.018228626171312
d = 0.1
CT0s =  1.538190483976698e-5
load_factor_limit = 1.5

pwm_max = 120
pwm_min = 0

data = 0
data_sensors = []

motor_running = True

z = 0.0
z_dot = 0.0
z_offset = 0.0

roll = 0.0
p = 0.0
pitch = 0.0
q = 0.0
yaw = 0.0
r = 0.0

A = np.array([[0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0]])

B = np.array([[0, 0, 0, 0],
              [1/m, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ixx, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 1/Iyy, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 1/Izz]])

C = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 1, 0]])

D = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

sys = control.ss(A, B, C, D)

Q = np.diag([100, 1, 100, 1, 100, 1, 1, 10])*1000
R = np.diag([1, 1, 1, 1])*.01
K, S, E = control.lqr(sys, Q, R)
print("K:",K)

syscl = control.ss(A-(B@K), B, C, D)
Kr = np.linalg.inv(control.dcgain(syscl))
print("Kr:",Kr)

u = np.array([0, 0, 0, 0], dtype=np.float64)
x = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
x_des = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
w = np.array([0, 0, 0, 0], dtype=np.uint8)

# M = np.array([[k, k, k, k],
#               [0, -l*k, 0, l*k],
#               [-l*k, 0, l*k, 0],
#               [-d, d, -d, d]], dtype=np.float64)
              
M = np.array([[1, 1, 1, 1],
              [0, -1, 0, 1],
              [-1, 0, 1, 0],
              [1, -1, 1, -1]], dtype=np.float64)
M = np.linalg.inv(M)
print('M:',M)

def engage_motor():
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()
    motor_running = True

def shutdown_motor():
    pwm = [0, 0, 0, 0]
    rospy.loginfo(pwm)
    pubsub.publish_pwm(pwm)
    rospy.wait_for_service("/shutdown")
    rospy.ServiceProxy("/shutdown", Empty)()
    motor_running = False

def close():
    if motor_running:
        shutdown_motor()
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()

def save(d):
    with open('data.csv', 'w') as f:
        write = csv.writer(f)
        data = [[v] for v in d]
        write.writerows(data)

def plot_data(d):
    figure, axis = plt.subplots(2, 2)
    figure.suptitle('Quadcopter Data')

    axis[0, 0].plot(d[0, :])
    axis[0, 0].set_title('z')
    axis[0, 0].set_xlabel('time')
    axis[0, 0].set_ylabel('z')
    axis[0, 0].grid(True)

    axis[0, 1].plot(d[1, :])
    axis[0, 1].set_title('roll')
    axis[0, 1].set_ylim(-np.pi, np.pi)
    axis[0, 1].set_xlabel('time')
    axis[0, 1].set_ylabel('roll')
    axis[0, 1].grid(True)

    axis[1, 0].plot(d[2, :])
    axis[1, 0].set_title('pitch')
    axis[1, 0].set_ylim(-np.pi, np.pi)
    axis[1, 0].set_xlabel('time')
    axis[1, 0].set_ylabel('pitch')
    axis[1, 0].grid(True)

    axis[1, 1].plot(d[3, :])
    axis[1, 1].set_title('yaw')
    axis[1, 1].set_ylim(-np.pi, np.pi)
    axis[1, 1].set_xlabel('time')
    axis[1, 1].set_ylabel('d_yaw')
    axis[1, 1].grid(True)

    plt.show()

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub, True)

    engage_motor()

    z_des = 1.0
    x_des[0] = z_des

    for i in range(500):
        rospy.loginfo("Data: {}".format(data))
        states.update_position()
        states.update_velocity()
        states.update_euler()
        states.update_ang_vel()

        z = states.ground_truth_pos.z
        z_dot = states.ground_truth_vel.z
        roll = states.roll
        p = states.p
        pitch = states.pitch
        q = states.q
        yaw = states.yaw
        r = states.r

        data_sensors.append([z, roll, pitch, r])

        x = np.array([z, z_dot, roll, p, pitch, q, yaw, r], dtype=np.float64)
        u = Kr @ (C @ x_des) - K @ x

        u[0] += (m*g)

        print("z: ", z)
        print("roll: ", roll)
        print("pitch: ", pitch)
        print("yaw: ", yaw)
        print("u: ", u)

        try:
            load_factor = 1 / (states.quaternion.w ** 2 - 
                            states.quaternion.x ** 2 -
                            states.quaternion.y ** 2 +
                            states.quaternion.z ** 2)
        except ZeroDivisionError:
            load_factor = 1.0
        
        if load_factor > load_factor_limit:
            load_factor = load_factor_limit

        if (not motor_running) and (u[0] > 0.1) and (load_factor > 0.0):
            motor_running = True
            engage_motor()
        elif (motor_running) and (u[0] < m*g):
            motor_running = False
            shutdown_motor()

        if motor_running and load_factor < 0.0:
            motor_running = False

        w2 = M @ u
        print("w2:", w2)
        
        if motor_running:
            for i in range(4):
                try:
                    w[i] = np.sqrt(w2[i])
                    w[i] = np.clip(w[i], pwm_min, pwm_max)
                except ValueError:
                    w[i] = 0

        pwm = w.tolist()
        print("pwm: ", pwm)
        pubsub.publish_pwm(pwm)

        print("Motor running: {}".format(motor_running))
        print("")
        data += 1
        rate.sleep()

    try:
        save(data_sensors)
    except:
        rospy.loginfo("Failed to save data")
    shutdown_motor()
    plot_data(np.array(data_sensors).T)
    close()