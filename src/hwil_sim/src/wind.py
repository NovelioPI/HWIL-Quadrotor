#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
import random

class Wind:
    def __init__(self):
        self.wind_pub = rospy.Publisher('/wind', Vector3, queue_size=1)
        self.wind = Vector3()
        self.wind_prev = Vector3()

        self.wind_prev.x = 0.0
        self.wind_prev.y = 0.0
        self.wind_prev.z = 0.0

        self.max_wind = 0 # m/s

    def update(self):
        delta = Vector3()

        delta.x = random.uniform(-1.0, 1.0)
        delta.y = random.uniform(-1.0, 1.0)
        delta.z = random.uniform(-1.0, 1.0)

        self.wind.x = self.wind.x + delta.x
        self.wind.y = self.wind.y + delta.y
        self.wind.z = self.wind.z + delta.z

        self.wind.x = max(min(self.wind.x, self.max_wind), -self.max_wind)
        self.wind.y = max(min(self.wind.y, self.max_wind), -self.max_wind)
        self.wind.z = max(min(self.wind.z, self.max_wind), -self.max_wind)

        rospy.loginfo("Wind: x: %f, y: %f, z: %f", self.wind.x, self.wind.y, self.wind.z)

        self.wind_pub.publish(self.wind)

if __name__ == '__main__':
    rospy.init_node('wind', anonymous=True)
    rate = rospy.Rate(10)

    wind = Wind()

    while not rospy.is_shutdown():
        wind.update()
        rate.sleep()