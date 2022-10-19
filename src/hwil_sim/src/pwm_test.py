#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from states import States
from pubsub import PubSub

def engage_motor():
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()

def shutdown_motor():
    pwm = [0, 0, 0, 0]
    rospy.loginfo(pwm)
    pubsub.publish_pwm(pwm)
    rospy.wait_for_service("/shutdown")
    rospy.ServiceProxy("/shutdown", Empty)()

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    rospy.on_shutdown(shutdown_motor)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub)

    engage_motor()

    while not rospy.is_shutdown():
        pwm = [250, 250, 250, 250]
        rospy.loginfo(pwm)
        pubsub.publish_pwm(pwm)
        rate.sleep()