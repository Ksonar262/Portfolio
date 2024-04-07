#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy 
from geometry_msgs.msg import Twist
import telnetlib
from Humanoid import Humanoid

class EZ_Robot():     

    def vel_callback(self, topic_message):
        self.Robot.move(topic_message)

    def __init__(self): 

        # Create a new instance of the Humanoid class
        self.Robot = Humanoid()

        self.node_name = "ez_robot"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        # Define the shutdown operations
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        rospy.loginfo(f"The '{self.node_name}' node is shutting down...")
        self.Robot.shut_down()

    def main_loop(self):
        rospy.spin() 

if __name__ == '__main__': 
    ez_instance = EZ_Robot()
    ez_instance.main_loop()
