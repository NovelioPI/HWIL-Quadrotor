#!/usr/bin/env python3

import rospy
import proto.hwil_pb2 as hwil_pb2
from pubsub import PubSub
from communication import Communication
from states import States
from std_srvs.srv import Empty

m = 1.477
g = 9.8065
motor_running = False

def calculate_load_factor(orientation):
    try:
        load_factor = 1 / (orientation.w ** 2 - 
                           orientation.x ** 2 -
                           orientation.y ** 2 +
                           orientation.z ** 2)
    except ZeroDivisionError:
        load_factor = 1.0
        
    return load_factor

def engage_motor():
    global motor_running
    print("Engaging motor")
    rospy.wait_for_service("/engage")
    rospy.ServiceProxy("/engage", Empty)()
    motor_running = True

def shutdown_motor():
    global motor_running
    print("Shutting down motor")
    pwm = [0, 0, 0, 0]
    rospy.loginfo(pwm)
    pubsub.publish_pwm(pwm)
    rospy.wait_for_service("/shutdown")
    rospy.ServiceProxy("/shutdown", Empty)()
    motor_running = False

def close():
    if motor_running:
        shutdown_motor()
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub, True)
    comm = Communication()

    engage_motor()

    print("Waiting for connection...")

    while not rospy.is_shutdown():        
        states.update_position()
        states.update_velocity()
        states.update_euler()
        states.update_ang_vel()

        rospy.loginfo("Position: {}".format(states.ground_truth_pos.z))

        msg = hwil_pb2.msg()

        msg.state.z = states.ground_truth_pos.z
        msg.state.roll = states.roll
        msg.state.pitch = states.pitch
        msg.state.yaw = states.yaw
        msg.state.vz = states.ground_truth_vel.z
        msg.state.p = states.p
        msg.state.q = states.q
        msg.state.r = states.r

        comm.write(msg)
        msg = comm.read()
        rospy.loginfo(msg)

        load_factor = calculate_load_factor(states.quaternion)
        
        if msg is not None:
            if (not motor_running) and (msg.command.u1 > 0.1) and (load_factor > 0.0):
                engage_motor()
            elif (motor_running) and (msg.command.u1 < m*g):
                shutdown_motor()
            
            if motor_running and load_factor < 0.0:
                shutdown_motor()
            
            pwm = [msg.command.motor1.pwm, msg.command.motor2.pwm, msg.command.motor3.pwm, msg.command.motor4.pwm]
            pubsub.publish_pwm(pwm)
        
        rate.sleep()