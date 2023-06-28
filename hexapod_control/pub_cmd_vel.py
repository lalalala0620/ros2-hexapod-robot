#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class control_pub(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.pub_twist) 	
        
    def pub_twist(self, vel_x=0.0, vel_y=1.0, vel_z=0.0, omega_x=0.0, omega_y=0.0, omega_z=0.0):
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = vel_z
        twist.angular.x = omega_x
        twist.angular.y = omega_y
        twist.angular.z = omega_z
        self.publisher_.publish(twist)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = control_pub("pub_vel")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("'Stopped by keyboard interrupt'")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()
