#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import numpy as np
import control
from states import States
from pubsub import PubSub
from time import sleep

m = 1.477
g = 9.8065
Ixx = 0.01152
Iyy = 0.01152
Izz = 0.0218
l = 0.275
k = 0.018228626171312
d = 0.1

omega2max = 120
omega2min = 0
sonar = 0

A = np.array([[0, 1],
              [0, 0]])

B = np.array([[0], [1/m]])

Q = np.array([[1000, 0],
              [0, 100]])

R = np.array([[0.1]])

K, S, E = control.lqr(A, B, Q, R)

def engage_motor():
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()

def shutdown_motor():
    pwm = [0, 0, 0, 0]
    rospy.loginfo(pwm)
    pubsub.publish_pwm(pwm)
    rospy.wait_for_service("/shutdown")
    rospy.ServiceProxy("/shutdown", Empty)()

def close():
    shutdown_motor()
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()

def map_value(value, new_min, new_max, old_min, old_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((value - old_min) * new_range) / old_range) + new_min

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub)

    engage_motor()

    motor_running = True

    z = 0.0
    z_dot = 0.0
    z_offset = 0.0
    z_des = 1.0

    x_des = np.array([z_des, 0], dtype=np.float64)

    for i in range(100):
        states.update_position()
        z_offset += states.z
        rate.sleep()
    z_offset /= 100

    load_factor_limit = 1.5

    w = 0

    while not rospy.is_shutdown():
        states.update_position()
        states.update_velocity()
        z = -(states.z - z_offset)
        z_dot = states.vel_z

        x = np.array([z, z_dot], dtype=np.float64)

        error = x - x_des
        
        u = -K @ error
        u = np.clip(u, -4, 4)

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
        elif (motor_running) and (u[0] < -0.1) and (error[0] > 0.0):
            motor_running = False
            shutdown_motor()

        if motor_running and load_factor < 0.0:
            motor_running = False
        
        if motor_running:
            try:
                w = u[0]/4
                w = round(map_value(w, omega2min, omega2max, -1, 1))
            except ValueError:
                w = 0

        pwm = [w, w, w, w]
        pubsub.publish_pwm(pwm)

        rospy.loginfo("Data:")
        print("Motor running: {}".format(motor_running))
        print("z: ", z)
        print("u: ", u)
        print("w: ", w)

        rate.sleep()