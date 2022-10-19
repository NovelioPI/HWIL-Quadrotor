#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Range
import numpy as np
import control
from states import States
from pubsub import PubSub
from time import sleep

m = 1.477
g = 9.81
Ixx = 0.01152
Iyy = 0.01152
Izz = 0.0218
l = 0.275
k = 0.018228626171312
d = 0.1

omega2max = 120
omega2min = 0

A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

B = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [1/m, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ixx, 0, 0],
              [0, 0, 1/Iyy, 0],
              [0, 0, 0, 1/Izz]])

Q = np.diag([1000, 1000, 1000, 10, 10, 10, 100, 100, 100, 10, 10, 1000])
R = np.diag([1, 1, 1, 1])*0.01
K, S, E = control.lqr(A, B, Q, R)
K = np.array(K)

u = np.array([0, 0, 0, 0], dtype=np.float64)
x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
x_hat = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
x_des = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

M = np.array([[k, k, k, k],
              [0, l*k, 0, -l*k],
              [-l*k, 0, l*k, 0],
              [d, -d, d, -d]])
M = np.linalg.inv(M)

pwm_max = [-83.52852477, 110.95790949, 115.95790949, -88.52852477]
pwm_min = [0.0, 0.0, 0.0, 0.0]

def engage_motor():
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()

def shutdown_motor():
    pwm = [0, 0, 0, 0]
    rospy.loginfo(pwm)
    pubsub.publish_pwm(pwm)
    rospy.wait_for_service("/shutdown")
    rospy.ServiceProxy("/shutdown", Empty)()
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()

def map_value(value, new_min, new_max, old_min, old_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((value - old_min) * new_range) / old_range) + new_min

def wrench_callback(msg):
    rospy.loginfo("force z: {}".format(msg.wrench.force.z))

def sonar_callback(msg):
    rospy.loginfo("range: {}".format(msg.range))

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/propulsion/wrench", WrenchStamped, wrench_callback)
    rospy.Subscriber("/sonar_height", Range, sonar_callback)
    rospy.on_shutdown(shutdown_motor)
    rate = rospy.Rate(250)
    pubsub = PubSub()
    states = States(pubsub)

    engage_motor()
    # rate.sleep()

    while not rospy.is_shutdown():
        states.update()
        x = states.get_state
        
        u = K @ (x - x_des)
        # u = np.clip(u, 0.0, 1.0)

        rospy.loginfo("u: {}".format(u))
        omega2 = M @ u
        pwm0 = np.sqrt(omega2[0])
        pwm1 = np.sqrt(omega2[1])
        pwm2 = np.sqrt(omega2[2])
        pwm3 = np.sqrt(omega2[3])
        pwm = np.array([pwm0, pwm1, pwm2, pwm3], dtype=np.uint8)

        rospy.loginfo("PWM: {}".format(pwm))
        pubsub.publish_pwm(pwm.tolist())
        rate.sleep()



