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

def sensors():
    _msg = hwil_pb2.msg()

    _msg.sensors.imu.gyro.x = pubsub.imu_msg.angular_velocity.x
    _msg.sensors.imu.gyro.y = pubsub.imu_msg.angular_velocity.y
    _msg.sensors.imu.gyro.z = pubsub.imu_msg.angular_velocity.z
    _msg.sensors.imu.accel.x = pubsub.imu_msg.linear_acceleration.x
    _msg.sensors.imu.accel.y = pubsub.imu_msg.linear_acceleration.y
    _msg.sensors.imu.accel.z = pubsub.imu_msg.linear_acceleration.z
    _msg.sensors.imu.mag.x = 0
    _msg.sensors.imu.mag.y = 0
    _msg.sensors.imu.mag.z = 0
    _msg.sensors.imu.orientation.w = pubsub.imu_msg.orientation.w
    _msg.sensors.imu.orientation.x = pubsub.imu_msg.orientation.x
    _msg.sensors.imu.orientation.y = pubsub.imu_msg.orientation.y
    _msg.sensors.imu.orientation.z = pubsub.imu_msg.orientation.z
    _msg.state.z = pubsub.ground_truth_msg.pose.pose.position.z
    _msg.state.vz = pubsub.ground_truth_msg.twist.twist.linear.z

    return _msg

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
    rospy.init_node('hwil_node', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub, True)
    comm = Communication()

    msg = hwil_pb2.msg()
    msg.system_state = 4

    print("Waiting for connection...")
    print("Calibrating..")
    while not msg.system_state == 0:
        msg = sensors()
        msg.system_state = 4

        comm.write(msg)
        msg = comm.read()

        rate.sleep()
    print("Calibration done")

    engage_motor()

    while not rospy.is_shutdown():

        msg = sensors()
        msg.system_state = 1

        comm.write(msg)
        rospy.loginfo("Sending message: {}".format(msg))
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