#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from states import States
from pubsub import PubSub

rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def shutdown():
    global pub
    cmd = Twist()
    cmd.linear.z = 0
    pub.publish(cmd)
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()

def callback(data):
    rospy.loginfo(data.altitude)

rospy.Subscriber('/fix', NavSatFix, callback)
rospy.on_shutdown(shutdown)

if __name__ == "__main__":
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub)

    cmd = Twist()

    while not rospy.is_shutdown():
        cmd.linear.z = 1
        pub.publish(cmd)
        rate.sleep()