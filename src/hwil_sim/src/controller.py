#!/usr/bin/env python3

import rospy
import numpy as np
import control
from states import States
from pubsub import PubSub

class Controller:
    def __init__(self, pubsub):
        self._states = States(pubsub)

        self._m = 1.477
        self._g = 9.81
        self._Ixx = 0.01152
        self._Iyy = 0.01152
        self._Izz = 0.0218
        self._l = 0.275
        self._d = 0.018228626171312
        self._k = 0.1

        self._omega2max = 255
        self._omega2min = 0

        self._A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, -self._g, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, self._g, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        
        self._B = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [1/self._m, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 1/self._Ixx, 0, 0],
                            [0, 0, 1/self._Iyy, 0],
                            [0, 0, 0, 1/self._Izz]])

        self._C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])

        self._D = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]])

        self._Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        self._R = np.diag([1, 1, 1, 1])

        K, X, E = control.lqr(self._A, self._B, self._Q, self._R)
        self._K = np.array(K)

    def full_state_feedback(self, desired):
        self._x = self._states.get_state
        u = -self._K.dot(self._x - desired)

        u0max = 4*self._k*self._omega2max
        u0min = 4*self._k*self._omega2min
        u12max = self._l*self._k*self._omega2max
        u12min = -self._l*self._k*self._omega2max
        u3max = 2*self._d*self._omega2max
        u3min = -2*self._d*self._omega2max

        u[0] = np.clip(u[0], u0min, u0max)
        u[1] = np.clip(u[1], u12min, u12max)
        u[2] = np.clip(u[2], u12min, u12max)
        u[3] = np.clip(u[3], u3min, u3max)

        return u
    
    def u_to_pwm(self, u):
        pwm = np.array([0, 0, 0, 0])
        _4K = 4*self._k
        _2lK = 2*self._l*self._k
        _4d = 4*self._d

        pwm[0] = u[0]/_4K + u[2]/_2lK + u[3]/_4d
        pwm[1] = u[0]/_4K - u[1]/_2lK - u[3]/_4d
        pwm[2] = u[0]/_4K - u[2]/_2lK + u[3]/_4d
        pwm[3] = u[0]/_4K + u[1]/_2lK - u[3]/_4d

        return pwm
        

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    controller = Controller(pubsub)

    if not rospy.is_shutdown():
        desired = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        u = controller.full_state_feedback(desired)
        pwm = controller.u_to_pwm(u)
        pubsub.publish_pwm(pwm)
        rate.sleep()



