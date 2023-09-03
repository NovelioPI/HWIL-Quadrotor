#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import numpy as np
import control
from states import States
from pubsub import PubSub
import csv
import matplotlib.pyplot as plt
from time import sleep

m = 1.477
g = 9.8065
# Ixx = 0.014641566666666668
# Iyy = 0.014641566666666668
# Izz = 0.026640733333333336
Ixx = 0.014891
Iyy = 0.015712
Izz = 0.027364271
l = 0.275
k = 0.018228626171312
d = 0.1
CT0s =  1.538190483976698e-5
load_factor_limit = 1.5

pwm_max = 250
pwm_min = 0

data = 0
data_sensors = []
data_plot = []

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

C = np.identity(8)

D = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

sys = control.ss(A, B, C, D)

integral_roll = 0.0
integral_pitch = 0.0

Ki_roll = 1.5
Ki_pitch = 1.5

# Q = np.diag([100, 1, 2500, 1700, 2000, 1300, 450, 30])
# Q = np.diag([100, 1, 25, 17, 20, 13, 4.5, 0.2]) # Simulation
Q = np.diag([100, 1, 25, 17, 30, 8, 9, 1]) # Simulation
# Q = np.diag([100, 1, 9, 11, 9, 11, 5, 0.7]) # Real
R = np.diag([1, 1, 1, 1])   
K, S, E = control.lqr(sys, Q, R)
print("K alt:",K[0, 0])
print("K vz:",K[0, 1])
print("K roll:",K[1, 2])
print("K p:",K[1, 3])
print("K pitch:",K[2, 4])
print("K q:",K[2, 5])
print("K yaw:",K[3, 6])
print("K r:",K[3, 7])

# syscl = control.ss(A-(B@K), B, C-(D@K), D)
# Kr = np.linalg.pinv(control.dcgain(syscl))
# print("Kr:",Kr)

x = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
x_des = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
u = np.array([0, 0, 0, 0], dtype=np.float64)
w = np.array([0, 0, 0, 0], dtype=np.uint16)

error_roll = 0.0
error_pitch = 0.0
error_yaw = 0.0

# M = np.array([[k, k, k, k],
#               [0, -l*k, 0, l*k],
#               [-l*k, 0, l*k, 0],
#               [d, -d, d, -d]], dtype=np.float64)
              
# M = np.array([[1, 1, 1, 1],
#               [0, -1, 0, 1],
#               [-1, 0, 1, 0],
#               [1, -1, 1, -1]], dtype=np.float64)
# M = np.linalg.inv(M)
# M = np.array([[0.25,  0.0, -0.5,  0.25],
#               [0.25, -0.5,  0.0, -0.25],
#               [0.25,  0.0,  0.5,  0.25],
#               [0.25,  0.5,  0.0, -0.25]], dtype=np.float64)

# M = np.array([[0.292600,  0.0000000, -0.1300300,  0.6283300],
#               [0.292600, -0.1300300,  0.0000000, -0.6283300],
#               [0.292600,  0.0000000,  0.1300300,  0.6283300],
#               [0.292600,  0.1300300,  0.0000000, -0.6283300]], dtype=np.float64)

M = np.array([[292600.0,  0.0000000, -1300300.0,  6283300.0],
              [292600.0, -1300300.0,  0.0000000, -6283300.0],
              [292600.0,  0.0000000,  1300300.0,  6283300.0],
              [292600.0,  1300300.0,  0.0000000, -6283300.0]], dtype=np.float64)

# print('M:',M)

def engage_motor():
    global motor_running
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()
    motor_running = True

def shutdown_motor():
    global motor_running
    pwm = [0, 0, 0, 0]
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
    with open('data_pitch2.csv', 'w') as f:
        write = csv.writer(f)
        data = [[v] for v in d]
        write.writerows(data)

def plot_data(d):
    figure, axis = plt.subplots(2, 4)
    figure.suptitle('Quadcopter Data')

    axis[0, 0].plot(d[0, :])
    axis[0, 0].plot(d[8, :])
    axis[0, 0].set_title('z')
    axis[0, 0].set_xlabel('time')
    axis[0, 0].set_ylabel('z')
    # axis[0, 0].axis(ymin=-2, ymax=2)
    axis[0, 0].grid(True)
    axis[0, 0].legend(['ground truth', 'sensor'])

    axis[0, 1].plot(d[1, :])
    axis[0, 1].plot(d[9, :])
    axis[0, 1].set_title('z_dot')
    axis[0, 1].set_xlabel('time')
    axis[0, 1].set_ylabel('z_dot')
    axis[0, 1].axis(ymin=-1, ymax=1)
    axis[0, 1].grid(True)
    axis[0, 1].legend(['ground truth', 'sensor'])

    axis[0, 2].plot(d[2, :])
    axis[0, 2].plot(d[10, :])
    axis[0, 2].set_title('roll')
    axis[0, 2].set_xlabel('time')
    axis[0, 2].set_ylabel('roll')
    axis[0, 2].axis(ymin=-30, ymax=30)
    axis[0, 2].grid(True)
    axis[0, 2].legend(['ground truth', 'sensor'])

    axis[0, 3].plot(d[3, :])
    axis[0, 3].plot(d[11, :])
    axis[0, 3].set_title('p')
    axis[0, 3].set_xlabel('time')
    axis[0, 3].set_ylabel('p')
    # axis[0, 3].axis(ymin=-1, ymax=1)
    axis[0, 3].grid(True)
    axis[0, 3].legend(['ground truth', 'sensor'])

    axis[1, 0].plot(d[4, :])
    axis[1, 0].plot(d[12, :])
    axis[1, 0].set_title('pitch')
    axis[1, 0].set_xlabel('time')
    axis[1, 0].set_ylabel('pitch')
    axis[1, 0].axis(ymin=-30, ymax=30)
    axis[1, 0].grid(True)
    axis[1, 0].legend(['ground truth', 'sensor'])

    axis[1, 1].plot(d[5, :])
    axis[1, 1].plot(d[13, :])
    axis[1, 1].set_title('q')
    axis[1, 1].set_xlabel('time')
    axis[1, 1].set_ylabel('q')
    # axis[1, 1].axis(ymin=-1, ymax=1)
    axis[1, 1].grid(True)
    axis[1, 1].legend(['ground truth', 'sensor'])

    axis[1, 2].plot(d[6, :])
    axis[1, 2].plot(d[14, :])
    axis[1, 2].set_title('yaw')
    axis[1, 2].set_xlabel('time')
    axis[1, 2].set_ylabel('yaw')
    axis[1, 2].axis(ymin=-30, ymax=30)
    axis[1, 2].grid(True)
    axis[1, 2].legend(['ground truth', 'sensor'])

    axis[1, 3].plot(d[7, :])
    axis[1, 3].plot(d[15, :])
    axis[1, 3].set_title('r')
    axis[1, 3].set_xlabel('time')
    axis[1, 3].set_ylabel('r')
    # axis[1, 3].axis(ymin=-1, ymax=1)
    axis[1, 3].grid(True)
    axis[1, 3].legend(['ground truth', 'sensor'])

    plt.show()

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub, True)
    sensor = States(pubsub, False)

    engage_motor()

    z_des = 100.0
    roll_des = 0.0
    pitch_des = 0.0
    yaw_des = 0.0
    
    euler_offset = np.array([0, 0, 0])
    gyro_offset = np.array([0, 0, 0])

    for i in range(100):
        sensor.update_euler()
        sensor.update_ang_vel()
        euler_offset = euler_offset + np.array([sensor.roll, sensor.pitch, sensor.yaw])
        gyro_offset = gyro_offset + np.array([sensor.p, sensor.q, sensor.r])
        rate.sleep()
    euler_offset = euler_offset / 100
    gyro_offset = gyro_offset / 100

    test_done = False

    for i in range(2000):
        sensor.update_euler()
        sensor.update_ang_vel()
        states.update_states()

        # if i > 500:
        #     z_des = 0
        #     if z < 5.0:
        #         break

        # if i > 1000 and not test_done:
        #     roll_des = -25.0
        #     if error_roll < 1.0 and i > 1010:
        #         roll_des = 0.0
        #         test_done = True
        
        if i > 1000 and not test_done:
            pitch_des = -25.0
            if error_pitch < 1.0 and i > 1010:
                pitch_des = 0.0
                test_done = True
        
        # if i > 1000 and not test_done:
        #     yaw_des = -25.0
        #     if error_yaw < 1.0 and i > 1010:
        #         yaw_des = 0.0
        #         test_done = True
                

        z = (states.z - 0.2) * 100
        z_dot = states.vel_z * 100
        roll = (sensor.roll - euler_offset[0]) * 180 / np.pi
        p = (sensor.p - gyro_offset[0]) * 180 / np.pi
        pitch = (sensor.pitch - euler_offset[1]) * 180 / np.pi
        q = (sensor.q - gyro_offset[1]) * 180 / np.pi
        yaw = (sensor.yaw - euler_offset[2]) * 180 / np.pi
        r = (sensor.r - gyro_offset[2]) * 180 / np.pi

        # data_sensors.append([z/100, z_dot/100, states.roll, states.p, states.pitch, states.q, states.yaw, states.r, 1, 0, roll, p, pitch, q, yaw, r])
        # data_sensors.append([pubsub.ground_truth_msg.pose.pose.position.z,
        #                      pubsub.imu_msg.orientation.w,
        #                      pubsub.imu_msg.orientation.x,
        #                      pubsub.imu_msg.orientation.y,
        #                      pubsub.imu_msg.orientation.z,
        #                      pubsub.imu_msg.angular_velocity.x,
        #                      pubsub.imu_msg.angular_velocity.y,
        #                      pubsub.imu_msg.angular_velocity.z,])
        data_sensors.append([pitch])
        data_plot.append([z/100, z_dot/100, states.roll, states.p, states.pitch, states.q, states.yaw, states.r, 1, 0, roll, p, pitch, q, yaw, r])

        error_roll = roll - roll_des
        error_pitch = pitch - pitch_des
        error_yaw = yaw - yaw_des

        if abs(error_roll) < 10.0:
            integral_roll += error_roll * 0.01
        else:
            integral_roll = 0.0
        if abs(error_pitch) < 10.0:
            integral_pitch += error_pitch * 0.01
        else:
            integral_pitch = 0.0

        u[0] = (-K[0,0] * (z - z_des) - K[0,1] * z_dot) / 5000000.0
        u[1] = (-K[1,2] * error_roll - K[1,3] * p - Ki_roll * integral_roll) / 5000000.0
        u[2] = (-K[2,4] * error_pitch - K[2,5] * q - Ki_pitch * integral_pitch) / 5000000.0
        u[3] = (-K[3,6] * error_yaw - K[3,7] * r) / 5000000.0

        integral_roll += error_roll * 0.01
        integral_pitch += error_pitch * 0.01

        try:
            load_factor = 1 / (states.quaternion.w ** 2 - 
                            states.quaternion.x ** 2 -
                            states.quaternion.y ** 2 +
                            states.quaternion.z ** 2)
        except ZeroDivisionError:
            load_factor = 1.0
        
        if load_factor > load_factor_limit:
            load_factor = load_factor_limit

        if (not motor_running) and (u[0]*5e6 > 0.0) and (load_factor > 0.0):
            engage_motor()
        elif (not motor_running) or (u[0]*5e6 < 0.1):
            shutdown_motor()

        if motor_running and load_factor < 0.0:
            motor_running = False
        
        if motor_running:
            w2 = np.zeros(4, dtype=np.float64)
            w2[0] = M[0,0] * u[0] + M[0,1] * u[1] + M[0,2] * u[2] + M[0,3] * u[3]
            w2[1] = M[1,0] * u[0] + M[1,1] * u[1] + M[1,2] * u[2] + M[1,3] * u[3]
            w2[2] = M[2,0] * u[0] + M[2,1] * u[1] + M[2,2] * u[2] + M[2,3] * u[3]
            w2[3] = M[3,0] * u[0] + M[3,1] * u[1] + M[3,2] * u[2] + M[3,3] * u[3]

            for i in range(4):
                w[i] = np.clip(w2[i], pwm_min, pwm_max)
        else:
            pwm = [0, 0, 0, 0]

        pwm = w.tolist()
        pubsub.publish_pwm(pwm)
        sleep(0.01)
        pubsub.publish_pwm([0, 0, 0, 0])

        rate.sleep()

    try:
        save(data_sensors)
    except:
        rospy.loginfo("Failed to save data")
    
    plot_data(np.array(data_plot).T)

    close()