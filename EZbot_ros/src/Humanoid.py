#! /usr/bin/env python3

# Control the robot using telnet commands
import telnetlib
from helpers import map_to_byte

host = "192.168.1.2"
port = 6666


class Humanoid:
    def __init__(self):
        # Connect to the telnet server
        self.connect_telnet()

    def connect_telnet(self):
        self.tn = telnetlib.Telnet(host, port)

    def close_telnet(self):
        self.tn.close()

    def shut_down(self):

        self.close_telnet()

    def move(self, topic_message):
        linear_x = map_to_byte(topic_message.linear.x)
        angular_z = map_to_byte(topic_message.angular.z)

        print (linear_x)
        print(angular_z)
        print("*********")

        if linear_x > 0:
            self.tn.write(b'Forward(' + str(-linear_x).encode("ascii") + b')\r\n')
        elif linear_x < 0:
            self.tn.write(b'Reverse(' + str(-linear_x).encode("ascii") + b')\r\n')
        elif angular_z > 0:
            self.tn.write(b'Right(' + str(-linear_x).encode("ascii") + b')\r\n')
        elif angular_z < 0:
            self.tn.write(b'Left(' + str(-linear_x).encode("ascii") + b')\r\n')
        else:
            self.tn.write(b'Stop()\r\n')