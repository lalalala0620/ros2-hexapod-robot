#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import sys
import numpy as np
import tf_transformations

class Joy_twist(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.pose = Pose()
        self.sub = self.create_subscription(Joy,'joy', self.listener_callback,0)
        self.publisher_twist = self.create_publisher(Twist, 'cmd_vel_joy', 0)
        self.publisher_pose = self.create_publisher(Pose, 'pose', 0)
        self.publisher_led = self.create_publisher(Float32MultiArray,'led_data', 0)
        
    def listener_callback(self, data):
        joy_data = data
        if joy_data.buttons[6] == 1:
            self.pub_cmdvel(joy_data)
            self.pub_pose(joy_data)
            self.pub_led(joy_data)
        
    def pub_led(self, joy_data):
        led_data = Float32MultiArray()
        led_data.data = []
        if joy_data.buttons[0] == 1:            
            led_data.data.append(0)
            led_data.data.append(0)
            led_data.data.append(255)
        elif joy_data.buttons[1] == 1:            
            led_data.data.append(0)
            led_data.data.append(255)
            led_data.data.append(0)
        elif joy_data.buttons[2] == 1:            
            led_data.data.append(255)
            led_data.data.append(0)
            led_data.data.append(0)
        elif joy_data.buttons[3] == 1:            
            led_data.data.append(0)
            led_data.data.append(0)
            led_data.data.append(0)

        self.publisher_led.publish(led_data)
            
    def pub_cmdvel(self, joy_data):
        twist = Twist()
        twist.linear.x = joy_data.axes[3]
        twist.linear.y = joy_data.axes[2]
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = joy_data.axes[0]
        self.publisher_twist.publish(twist)
        
    def pub_pose(self, joy_data):
        if joy_data.axes[4] == -1.0 and self.pose.position.x <= 1:
            self.pose.position.x += 0.1
        if joy_data.axes[4] == 1.0 and self.pose.position.x >= -1:
            self.pose.position.x -= 0.1

        if joy_data.axes[5] == 1.0 and self.pose.position.y <= 1:
            self.pose.position.y += 0.1
        elif joy_data.axes[5] == -1.0 and self.pose.position.y >= -1:
            self.pose.position.y -= 0.1
            
        if joy_data.buttons[8] == 1 and self.pose.position.z <= 1:
            self.pose.position.z += 0.1
        elif joy_data.buttons[9] == 1 and self.pose.position.z >= -1:
            self.pose.position.z -= 0.1
            
        if joy_data.buttons[7] == 1:
            roll = -joy_data.axes[0]*np.pi/2
            pitch = joy_data.axes[1]*np.pi/2
            yaw = 0
            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]
        self.publisher_pose.publish(self.pose)
        
def main(args=None):
    rclpy.init(args=args)
    node = Joy_twist("joy_twist")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__== '__main__':
    main()