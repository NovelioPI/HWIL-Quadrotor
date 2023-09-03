#!/usr/bin/env python3

import rospy
import numpy as np
import proto.hwil_pb2 as hwil_pb2
from pubsub import PubSub
from communication import Communication
from states import States
from std_srvs.srv import Empty
import control
import csv
from time import sleep

m = 1.477
g = 9.8065
Ixx = 0.014891;
Iyy = 0.015712;
Izz = 0.027364271;

motor_running = False

A = np.array([[0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0]])

B = np.array([[0, 0, 0, 0],
              [1/m, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ixx, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 1/Iyy, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 1/Izz]])

C = np.identity(8)

D = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

data_sensors = []
data_loss = 0

sys = control.ss(A, B, C, D)

Q = np.diag([100, 1, 25, 17, 20, 13, 4.5, 0.2]) # Simulation
# Q = np.diag([100, 1, 9, 11, 9, 11, 5, 0.7]) # Real
R = np.diag([1, 1, 1, 1])   
K, S, E = control.lqr(sys, Q, R)
print("K alt:",K[0, 0])
print("K vz:",K[0, 1])
print("K roll:",K[1, 2])
print("K p:",K[1, 3])
print("K pitch:",K[2, 4])
print("K q:",K[2, 5])
print("K yaw:",K[3, 6])
print("K r:",K[3, 7])

def sensors():
    _msg = hwil_pb2.msg()

    _msg.sensors.imu.gyro.x = pubsub.imu_msg.angular_velocity.x
    _msg.sensors.imu.gyro.y = pubsub.imu_msg.angular_velocity.y
    _msg.sensors.imu.gyro.z = pubsub.imu_msg.angular_velocity.z
    # _msg.sensors.imu.gyro.x = pubsub.ground_truth_msg.twist.twist.angular.x
    # _msg.sensors.imu.gyro.y = pubsub.ground_truth_msg.twist.twist.angular.y
    # _msg.sensors.imu.gyro.z = pubsub.ground_truth_msg.twist.twist.angular.z
    _msg.sensors.imu.accel.x = 0
    _msg.sensors.imu.accel.y = 0
    _msg.sensors.imu.accel.z = 0
    _msg.sensors.imu.mag.x = 0
    _msg.sensors.imu.mag.y = 0
    _msg.sensors.imu.mag.z = 0
    _msg.sensors.imu.orientation.w = pubsub.imu_msg.orientation.w
    _msg.sensors.imu.orientation.x = pubsub.imu_msg.orientation.x
    _msg.sensors.imu.orientation.y = pubsub.imu_msg.orientation.y
    _msg.sensors.imu.orientation.z = pubsub.imu_msg.orientation.z
    # _msg.sensors.imu.orientation.w = pubsub.ground_truth_msg.pose.pose.orientation.w
    # _msg.sensors.imu.orientation.x = pubsub.ground_truth_msg.pose.pose.orientation.x
    # _msg.sensors.imu.orientation.y = pubsub.ground_truth_msg.pose.pose.orientation.y
    # _msg.sensors.imu.orientation.z = pubsub.ground_truth_msg.pose.pose.orientation.z
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
    shutdown_motor()
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)()
    rospy.loginfo("data lost: %d", data_loss)

def save(d):
    with open('data_sensors.csv', 'w') as f:
        write = csv.writer(f)
        data = [[v] for v in d]
        write.writerows(data)

if __name__ == '__main__':
    rospy.init_node('hwil_node', anonymous=True)
    rospy.on_shutdown(close)
    rate = rospy.Rate(100)
    pubsub = PubSub()
    states = States(pubsub)
    comm = Communication(baudrate=6000000)

    msg = hwil_pb2.msg()
    msg.system_state = 0
    msg.gain.z = K[0, 0]
    msg.gain.vz = K[0, 1]
    msg.gain.roll = K[1, 2]
    msg.gain.p = K[1, 3]
    msg.gain.pitch = K[2, 4]
    msg.gain.q = K[2, 5]
    msg.gain.yaw = K[3, 6]
    msg.gain.r = K[3, 7]
    comm.write(msg)

    engage_motor()

    while not rospy.is_shutdown():

        msg = sensors()
        msg.system_state = 1

        comm.write(msg)

        msg = comm.read()

        try:
            rospy.loginfo("%d %d %d %d", msg.command.motor1.pwm, msg.command.motor2.pwm, msg.command.motor3.pwm, msg.command.motor4.pwm)
            data_sensors.append([msg.state.z, msg.state.roll, msg.state.pitch, msg.state.yaw])
            save(data_sensors)
        except:
            data_loss += 1
            pass

        load_factor = calculate_load_factor(states.quaternion)
        
        if (not motor_running) and (msg.command.u1 * 5e6 > 0.0) and (load_factor > 0.0):
            engage_motor()
        elif (not motor_running) or (msg.command.u1 * 5e6 < 0.1):
            shutdown_motor()

        if motor_running and load_factor < 0.0:
            motor_running = False
            
        pwm = [msg.command.motor1.pwm, msg.command.motor2.pwm, msg.command.motor3.pwm, msg.command.motor4.pwm]
        pubsub.publish_pwm(pwm)
        sleep(0.01)
        pubsub.publish_pwm([0, 0, 0, 0])

        rate.sleep()