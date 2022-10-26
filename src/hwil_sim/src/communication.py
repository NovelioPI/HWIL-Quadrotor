#!/usr/bin/env python3

import proto.hwil_pb2 as hwil_pb2
from serial import Serial

class Communication:
    def __init__(self, port = '/dev/ttyACM0', baudrate = 115200):
        self._serial = Serial(port, baudrate)

    def read(self):
        buffer = b''
        message = hwil_pb2.msg()
        while True:
            data = self._serial.read()
            if data == b'\r':
                data = self._serial.read() 
                if data == b'\n':
                    try:
                        message.ParseFromString(buffer)
                        return message
                    except:
                        print("Error decoding message")
                        break
                else:
                    buffer += b'\r' + data
            else:
                buffer += data

    def write(self, message):
        buffer = message.SerializeToString() + b'\r\n'
        self._serial.write(buffer)
        self._serial.flush()

# if __name__ == "__main__":
#     com = Communication(baudrate=9600)
#     while True:
#         msg = hwil_pb2.msg()
#         msg.state.x = 5.4

#         com.write(msg)

#         msg = com.read()
#         print(msg)
            