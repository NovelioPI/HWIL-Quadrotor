#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
import numpy as np
import random

class Wind:
    def __init__(self):
        self.wind_pub = rospy.Publisher('/wind', Vector3, queue_size=1)
        self.wind = Vector3()

        self.ver_angle = random.uniform(-np.pi, np.pi)
        self.hor_angle = random.uniform(-np.pi, np.pi)

    def update(self, mean_v, A, t):
        V = mean_v * (1 + A*(np.sin(0.5*t)*np.cos(0.25*t)*np.sin(0.35*t)*np.cos(t)*np.sin(0.1*t))) * 0.1

        self.wind.x = V * np.cos(self.ver_angle) * np.cos(self.hor_angle)
        self.wind.y = V * np.cos(self.ver_angle) * np.sin(self.hor_angle)
        self.wind.z = V * np.sin(self.ver_angle)

        self.wind_pub.publish(self.wind)

if __name__ == '__main__':
    rospy.init_node('wind', anonymous=True)
    rate = rospy.Rate(10)

    wind = Wind()

    while not rospy.is_shutdown():
        t = rospy.get_time()
        wind.update(1, 0.1, t)
        rate.sleep()