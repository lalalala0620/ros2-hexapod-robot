#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

sys.path.append('/home/spider/hexapod_ws/src/hexapod/hexapod/control')
from Control import *

#Creating object 'control' of 'Control' class.
c=Control()
class control_sub(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(Twist,'cmd_vel', self.listener_callback,0)

    def listener_callback(self, data):
        self.twist = data
        vel_x = self.twist.linear.x
        vel_y = self.twist.linear.y
        vel_z = self.twist.linear.z
        omega_x = self.twist.angular.x
        omega_y = self.twist.angular.y
        omega_z = self.twist.angular.z
        print(vel_x, vel_y, vel_z)
        print(omega_x, omega_y, omega_z)
        
        self.data=['CMD_MOVE', '1', vel_x, vel_y, '10', omega_z]
        c.run(self.data)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = control_sub("sub_vel")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()