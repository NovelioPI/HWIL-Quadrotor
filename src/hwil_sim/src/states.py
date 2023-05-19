#!/usr/bin/env python3

import rospy
# from pubsub import PubSub
import math
import numpy as np

class States:
    def __init__(self, pubsub, use_ground_truth=False):
        self._pubsub = pubsub
        self._pos = [0, 0, 0]
        self._vel = [0, 0, 0]
        self._acc = [0, 0, 0]
        self._euler = [0, 0, 0]
        self._ang_vel = [0, 0, 0]
        self._ang_acc = [0, 0, 0]
        self._state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self._pos_prev = [0, 0, 0]
        self._ang_vel_prev = [0, 0, 0]

        # Kalman filter
        self._kalman_first = True
        self._Xhat = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self._Pk = np.eye(6)

        # Moving average filter
        self._sum = [0, 0, 0]
        self._buffer = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self._index = 0
        self._window = 5

        self._gravity = [0, 0, 0]

        self._use_ground_truth = use_ground_truth

    def update_states(self):
        self.update_position()
        self.update_velocity()
        self.update_euler()
        self.update_ang_vel()

        self._state = self._pos + self._vel + self._euler + self._ang_vel

    def update_sonar(self, dt=0.1, use_derivative=True):
        self._pos[2] = self._pubsub.sonar_msg.range

        if self._pos[2] == self._pos_prev[2]:
            self._pos[2] += self._vel[2] * 0.01
            self._vel[2] = (self._pos[2] - self._pos_prev[2]) / 0.01
        elif use_derivative:
            self._vel[2] = (self._pos[2] - self._pos_prev[2]) / dt

        self._pos_prev[2] = self._pos[2]

    def update_position(self):
        if self._use_ground_truth:
            self._pos = [self._pubsub.ground_truth_msg.pose.pose.position.x, self._pubsub.ground_truth_msg.pose.pose.position.y, self._pubsub.ground_truth_msg.pose.pose.position.z]
        else:
            self._gps_to_xyz(self._pubsub.gps_msg)

    def update_velocity(self):
        if self._use_ground_truth:
            self._vel = [self._pubsub.ground_truth_msg.twist.twist.linear.x, self._pubsub.ground_truth_msg.twist.twist.linear.y, self._pubsub.ground_truth_msg.twist.twist.linear.z]
        else:
            self._acc = self._to_body(self._pubsub.imu_msg.orientation, self._pubsub.imu_msg.linear_acceleration)

            self._vel[0] = self._pubsub.velocity_msg.vector.x + 0.01 * self._acc[0]
            self._vel[1] = self._pubsub.velocity_msg.vector.y + 0.01 * self._acc[1]
            self._vel[2] = self._pubsub.velocity_msg.vector.z + 0.01 * self._acc[2]
    
    def update_acceleration(self):
        self._acc = self._to_body(self._pubsub.imu_msg.orientation, self._pubsub.imu_msg.linear_acceleration)

    def update_euler(self, use_quaternion=True):
        if self._use_ground_truth:
            self._quaternion_to_rpy(self._pubsub.ground_truth_msg.pose.pose.orientation)
        else:
            if use_quaternion:
                self._quaternion_to_rpy(self._pubsub.imu_msg.orientation)
            else:
                ax = self._pubsub.imu_msg.linear_acceleration.x
                ay = self._pubsub.imu_msg.linear_acceleration.y
                az = self._pubsub.imu_msg.linear_acceleration.z
                p = self._pubsub.imu_msg.angular_velocity.x
                q = self._pubsub.imu_msg.angular_velocity.y
                r = self._pubsub.imu_msg.angular_velocity.z
                self._phi = p * 0.01
                self._theta = q * 0.01
                self._psi = r * 0.01

                phi_dot_g = p + q * math.sin(self._phi) * math.tan(self._theta) + r * math.cos(self._phi) * math.tan(self._theta)
                theta_dot_g = q * math.cos(self._phi) - r * math.sin(self._phi)
                psi_dot_g = q * math.sin(self._phi) / math.cos(self._theta) + r * math.cos(self._phi) / math.cos(self._theta)

                phi_a = math.atan2(ay, az)
                theta_a = math.atan2(-ax, ay*math.sin(phi_a) + az*math.cos(phi_a))
                
                phi = 0.1 * phi_a + 0.9 * phi_dot_g * 0.01
                theta = 0.1 * theta_a + 0.9 * theta_dot_g * 0.01
                psi = psi_dot_g * 0.01

                self._euler = [phi, theta, psi]

    def update_ang_vel(self, use_kalman=False):
        if self._use_ground_truth:
            self._ang_vel = [self._pubsub.ground_truth_msg.twist.twist.angular.x, self._pubsub.ground_truth_msg.twist.twist.angular.y, self._pubsub.ground_truth_msg.twist.twist.angular.z]
        else:
            if use_kalman:
                self._ang_vel_prev = self._ang_vel
                self._ang_vel = [self._pubsub.imu_msg.angular_velocity.x, self._pubsub.imu_msg.angular_velocity.y, self._pubsub.imu_msg.angular_velocity.z]
                self._gyro_kalman_filter()
            else:
                self._ang_vel = [self._pubsub.imu_msg.angular_velocity.x, self._pubsub.imu_msg.angular_velocity.y, self._pubsub.imu_msg.angular_velocity.z]
                self._gyro_moving_average()
                
    def _quaternion_to_gravity(self, q):
        self._gravity[0] = 2 * (q.x*q.z - q.w*q.y)
        self._gravity[1] = 2 * (q.w*q.x + q.y*q.z)
        self._gravity[2] = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
        
    def _quaternion_to_rpy(self, q):
        self._quaternion_to_gravity(q)

        roll = math.atan2(self._gravity[1], self._gravity[2])
        pitch = math.atan2(self._gravity[0], math.sqrt(self._gravity[1]**2 + self._gravity[2]**2))
        yaw = math.atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1)

        if self._gravity[2]  < 0:
            if pitch > 0:
                pitch = math.pi - pitch 
            else:
                pitch = -math.pi - pitch
        
        self._euler = [roll, -pitch, -yaw]
    
    def _quaternion_to_euler(self, q):
        roll = math.atan2(2*q.y*q.z - 2*q.w*q.x, 2*q.w*q.w + 2*q.z*q.z - 1)
        pitch = math.asin(2*q.x*q.z + 2*q.w*q.y)
        yaw = math.atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1)

        self._euler = [-roll, pitch, -yaw]
    
    def _gps_to_xyz(self, gps):
        lat = self._pubsub.gps_msg.latitude * math.pi / 180
        lon = self._pubsub.gps_msg.longitude * math.pi / 180
        alt = self._pubsub.gps_msg.altitude

        lat0 = -7.7651543 * math.pi / 180
        lon0 = 110.3666396 * math.pi / 180
        alt0 = 0.0

        R = 6371000.0
        f = 1.0/298.257223563
        Reb = R * (1 - f)
        ecc = math.sqrt(R**2 - Reb**2) / R
        Ne = R / math.sqrt(1 - ecc**2 * math.sin(lat)**2)
        Neref = R / math.sqrt(1 - ecc**2 * math.sin(lat0)**2)

        # ECEF
        xe = (Ne + alt) * math.cos(lat) * math.cos(lon)
        ye = (Ne + alt) * math.cos(lat) * math.sin(lon)
        ze = (Ne * (1 - ecc**2) + alt) * math.sin(lat)

        # ECEF reference
        xeref = (Neref + alt0) * math.cos(lat0) * math.cos(lon0)
        yeref = (Neref + alt0) * math.cos(lat0) * math.sin(lon0)
        zeref = (Neref * (1 - ecc**2) + alt0) * math.sin(lat0)

        # NED
        Rne = np.array([[-math.sin(lat0)*math.cos(lon0), -math.sin(lat0)*math.sin(lon0), math.cos(lat0)], [-math.sin(lon0), math.cos(lon0), 0], [-math.cos(lat0)*math.cos(lon0), -math.cos(lat0)*math.sin(lon0), -math.sin(lat0)]])  # NED to ECEF
        x = Rne[0, 0] * (xe - xeref) + Rne[0, 1] * (ye - yeref) + Rne[0, 2] * (ze - zeref)  # NED x
        y = Rne[1, 0] * (xe - xeref) + Rne[1, 1] * (ye - yeref) + Rne[1, 2] * (ze - zeref)  # NED y
        z = Rne[2, 0] * (xe - xeref) + Rne[2, 1] * (ye - yeref) + Rne[2, 2] * (ze - zeref)  # NED z

        self._pos = [x, y, -z]  # NED

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

    def _gyro_kalman_filter(self):
        delta = np.array(self._ang_vel) - np.array(self._ang_vel_prev)

        Xk = np.array([self._ang_vel[0], delta[0], self._ang_vel[1], delta[1], self._ang_vel[2], delta[2]])
        A = np.array([[1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 1, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 0, 1]])
        Hk = np.eye(6)

        Wk = np.eye(6)*0.0001 # Q
        Vk = np.eye(6)*1e-10 # R

        if self._kalman_first:
            # one step prediction
            self._Xhat = np.dot(A, self._Xhat)

            # one step covariance
            self._Pk = np.dot(np.dot(A, self._Pk), A.T) + Wk

            self._kalman_first = False
        else:
            # kalman gain
            Kk = np.dot(np.dot(self._Pk, Hk.T), np.linalg.inv(np.dot(np.dot(Hk, self._Pk), Hk.T) + Vk))

            # update estimate
            self._Xhat = self._Xhat + np.dot(Kk, (Xk - np.dot(Hk, self._Xhat)))

            # update covariance
            self._Pk = np.dot((np.eye(6) - np.dot(Kk, Hk)), self._Pk)

        self._ang_vel = [self._Xhat[0], self._Xhat[2], self._Xhat[4]]
    
    def _gyro_moving_average(self):
        self._sum = [self._sum[i] - self._buffer[self._index][i] for i in range(3)]
        self._buffer[self._index] = self._ang_vel
        self._sum = [self._sum[i] + self._buffer[self._index][i] for i in range(3)]
        self._index = (self._index + 1) % self._window
        self._ang_vel = [self._sum[0] / self._window, self._sum[1] / self._window, self._sum[2] / self._window]
    
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

    @property
    def ground_truth_pos(self):
        return self._pubsub.ground_truth_msg.pose.pose.position
    
    @property
    def ground_truth_vel(self):
        return self._pubsub.ground_truth_msg.twist.twist.linear

# if __name__ == "__main__":
#     rospy.init_node("states")
#     rate = rospy.Rate(10)
#     pubsub = PubSub()
#     states = States(pubsub)
#     while not rospy.is_shutdown():
#         states.update()
#         print(states.get_state)
#         rate.sleep()