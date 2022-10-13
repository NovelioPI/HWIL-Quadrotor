#!/usr/bin/env python3

import rospy
import math
from pubsub import PubSub

class States:
    def __init__(self):
        self._pubsub = PubSub()
        self._pos = [0, 0, 0]
        self._vel = [0, 0, 0]
        self._euler = [0, 0, 0]
        self._ang_vel = [0, 0, 0]
        self._state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def update(self):
        # Update position
        self._gps_to_xyz(self._pubsub.gps_msg)

        # Update velocity
        self._vel = [self._pubsub.velocity_msg.vector.x, self._pubsub.velocity_msg.vector.y, self._pubsub.velocity_msg.vector.z]
        
        # Update Euler angles
        self._quaternion_to_euler(self._pubsub.imu_msg.orientation)

        # Update angular velocity
        self._ang_vel = [self._pubsub.imu_msg.angular_velocity.x, self._pubsub.imu_msg.angular_velocity.y, self._pubsub.imu_msg.angular_velocity.z]

        self._state = self._pos + self._vel + self._euler + self._ang_vel
    
    def _quaternion_to_euler(self, q):
        w = q.w
        x = q.x
        y = q.y
        z = q.z

        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        self._euler = [roll, pitch, yaw]
    
    def _gps_to_xyz(self, gps):
        lat = self._pubsub.gps_msg.latitude
        lon = self._pubsub.gps_msg.longitude
        alt = self._pubsub.gps_msg.altitude

        # ECEF
        Rn = 1 / math.sqrt(1 - 0.00669437999014 * math.sin(lat) * math.sin(lat))
        x = (Rn + alt) * math.cos(lat) * math.cos(lon)
        y = (Rn + alt) * math.cos(lat) * math.sin(lon)
        z = (Rn * (1 - 0.00669437999014) + alt) * math.sin(lat)

        self._pos = [x, y, z]
    
    @property
    def state(self):
        return self._state

if __name__ == "__main__":
    rospy.init_node("states")
    rate = rospy.Rate(10)
    states = States()
    while not rospy.is_shutdown():
        states.update()
        print(states.state)
        rate.sleep()