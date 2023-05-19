#!/usr/bin/env python3

import rospy
import proto.hwil_pb2 as hwil_pb2
from serial import Serial
from time import sleep
import codecs

# message format: STX + message size (2 bytes) + message + checksum

STX = b'\xFE'

class Communication:
    def __init__(self, port = '/dev/ttyACM0', baudrate = 9600):
        self._serial = Serial(port, baudrate)

    def read(self):
        buffer = b''
        checksum = 0
        message = hwil_pb2.msg()
        while self._serial.in_waiting > 3:
            data = self._serial.read()
            if data == STX:
                message_size = self._serial.read(1)[0] << 8 | self._serial.read(1)[0]
                for i in range(message_size):
                    data = self._serial.read(1)
                    buffer += data
                    checksum ^= data[0]
                if checksum == self._serial.read(1)[0]:
                    message.ParseFromString(buffer)
                    return message
                else:
                    rospy.logerr("Checksum error")
                    return None
        # rospy.logerr("Message too short")
        return None

    def write(self, message):
        buffer = message.SerializeToString()
        message_length = len(buffer)

        self._serial.write(STX)
        self._serial.write((message_length >> 8).to_bytes(1, 'big'))
        self._serial.write((message_length & 0xFF).to_bytes(1, 'big'))
        self._serial.write(buffer)

        checksum = 0x00
        for i in range(message_length):
            checksum ^= buffer[i]
        cb = checksum.to_bytes(1, 'big')
        self._serial.write(cb)
        self._serial.flush()

# if __name__ == "__main__":
#     com = Communication(baudrate=9600)
#     hwil = hwil_pb2.msg()
#     while True:
#         hwil.sensors.sonar.distance = 122345
#         hwil.system_state = 1
#         com.write(hwil)
#         sleep(1)

            