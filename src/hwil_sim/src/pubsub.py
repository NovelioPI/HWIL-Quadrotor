#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from hector_uav_msgs.msg import MotorPWM
from time import sleep

class PubSub:
    def __init__(self):
        self._imu_msg = Imu()
        self._sonar_msg = Range()
        self._ground_truth_msg = Odometry()
        self._motor_msg = MotorPWM()

        self._imu_sub = rospy.Subscriber("/raw_imu", Imu, self._imu_callback)
        self._sonar_sub = rospy.Subscriber("/sonar_height", Range, self._sonar_callback)
        self._ground_truth_sub = rospy.Subscriber("/ground_truth/state", Odometry, self._ground_truth_callback)
        
        self._motor_pub = rospy.Publisher("/motor_pwm", MotorPWM, queue_size=10)
    
    def publish_pwm(self, pwm):
        self._motor_msg.header.stamp = rospy.Time.now()
        self._motor_msg.header.frame_id = "base_link"
        self._motor_msg.pwm = pwm
        self._motor_pub.publish(self._motor_msg)

    def _imu_callback(self, msg):
        self._imu_msg = msg

    def _sonar_callback(self, msg):
        self._sonar_msg = msg

    def _ground_truth_callback(self, msg):
        self._ground_truth_msg = msg

    @property
    def imu_msg(self):
        return self._imu_msg

    @property
    def sonar_msg(self):
        return self._sonar_msg

    @property
    def ground_truth_msg(self):
        return self._ground_truth_msg

# if __name__ == "__main__":
#     rospy.init_node("pubsub")
#     rate = rospy.Rate(100)
#     pubsub = PubSub()
#     while not rospy.is_shutdown():
#         pwm = [150, 150, 150, 150]
#         pubsub.publish_pwm(pwm)
#         rate.sleep()