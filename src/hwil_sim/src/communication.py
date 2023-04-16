#!/usr/bin/env python3

import rospy
import proto.hwil_pb2 as hwil_pb2
from serial import Serial
from time import sleep

STX = b'\xFE'

class Communication:
    def __init__(self, port = '/dev/ttyACM0', baudrate = 115200):
        while(True):
            try:
                self._serial = Serial(port, baudrate)
                rospy.loginfo("Connected to " + port)
                break
            except:
                pass

    def read(self):
        buffer = b''
        checksum = 0
        message = hwil_pb2.msg()
        while True:
            data = self._serial.read(1)
            if data == STX:
                message_size = self._serial.read(1)[0] << 8 | self._serial.read(1)[0]
                for i in range(message_size):
                    data = self._serial.read(1)
                    buffer += data
                    checksum ^= data[0]
                if checksum == self._serial.read(1)[0]:
                    message.ParseFromString(buffer)
                    return message

    def write(self, message):
        buffer = message.SerializeToString()
        checksum = 0
        self._serial.write(STX)
        self._serial.write((len(buffer) >> 8).to_bytes(1, 'big'))
        self._serial.write((len(buffer) & 0xFF).to_bytes(1, 'big'))
        for i in range(len(buffer)):
            self._serial.write(buffer[i:i+1])
            checksum ^= buffer[i]
        self._serial.write(checksum.to_bytes(1, 'big'))

if __name__ == "__main__":
    com = Communication(baudrate=9600)
    hwil = hwil_pb2.msg()
    while True:
        hwil.sensors.sonar.distance = 122345
        hwil.system_state = 1
        com.write(hwil)
        sleep(1)

            