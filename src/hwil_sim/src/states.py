#!/usr/bin/env python3

import rospy
# from pubsub import PubSub
import math
import numpy as np

class States:
    def __init__(self, pubsub):
        self._pubsub = pubsub
        self._pos = [0, 0, 0]
        self._vel = [0, 0, 0]
        self._acc = [0, 0, 0]
        self._euler = [0, 0, 0]
        self._ang_vel = [0, 0, 0]
        self._ang_acc = [0, 0, 0]
        self._state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def update_state(self):
        self.update_position()
        self.update_velocity()
        self.update_euler()
        self.update_ang_vel()

        self._state = self._pos + self._vel + self._euler + self._ang_vel

    def update_position(self):
        self._gps_to_xyz(self._pubsub.gps_msg)

    def update_velocity(self):
        self._vel = [self._pubsub.velocity_msg.vector.x, self._pubsub.velocity_msg.vector.y, self._pubsub.velocity_msg.vector.z]
    
    def update_acceleration(self):
        self._acc = self._to_body(self._pubsub.imu_msg.orientation, self._pubsub.imu_msg.linear_acceleration)

    def update_euler(self):
        self._quaternion_to_euler(self._pubsub.imu_msg.orientation)

    def update_ang_vel(self):
        self._ang_vel = [self._pubsub.imu_msg.angular_velocity.x, self._pubsub.imu_msg.angular_velocity.y, self._pubsub.imu_msg.angular_velocity.z]
    
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

    def _to_body(self, q, nav):
        w = q.w
        x = q.x
        y = q.y
        z = q.z

        body = [0, 0, 0]

        body[0] =  (w*w+x*x-y*y-z*z) * nav.x + (2.*x*y + 2.*w*z) * nav.y + (2.*x*z - 2.*w*y) * nav.z
        body[1] =  (2.*x*y - 2.*w*z) * nav.x + (w*w-x*x+y*y-z*z) * nav.y + (2.*y*z + 2.*w*x) * nav.z
        body[2] =  (2.*x*z + 2.*w*y) * nav.x + (2.*y*z - 2.*w*x) * nav.y + (w*w-x*x-y*y+z*z) * nav.z

        return body
    
    @property
    def get_state(self):
        return self._state
    
    @property
    def quaternion(self):
        return self._pubsub.imu_msg.orientation
    
    @property
    def x(self):
        return self._pos[0]
    
    @property
    def y(self):
        return self._pos[1]

    @property
    def z(self):
        return self._pos[2]

    @property
    def vel_x(self):
        return self._vel[0]

    @property
    def vel_y(self):
        return self._vel[1]

    @property
    def vel_z(self):
        return self._vel[2]

    @property
    def acc_x(self):
        return self._acc[0]

    @property
    def acc_y(self):
        return self._acc[1]

    @property
    def acc_z(self):
        return self._acc[2]

    @property
    def roll(self):
        return self._euler[0]

    @property
    def pitch(self):
        return self._euler[1]

    @property
    def yaw(self):
        return self._euler[2]

    @property
    def p(self):
        return self._ang_vel[0]

    @property
    def q(self):
        return self._ang_vel[1]

    @property
    def r(self):
        return self._ang_vel[2]

# if __name__ == "__main__":
#     rospy.init_node("states")
#     rate = rospy.Rate(10)
#     pubsub = PubSub()
#     states = States(pubsub)
#     while not rospy.is_shutdown():
#         states.update()
#         print(states.get_state)
#         rate.sleep()