#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import sys
sys.path.append('/home/spider/hexapod_ws/src/hexapod/hexapod/control')
import time
import RPi.GPIO as GPIO
from Ultrasonic import *
    
ultrasonic = Ultrasonic()
    
class pub_distance(Node):    
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Range,'distance', 0)
        self.timer = self.create_timer(0.01, self.pub_distance) 
        
    def pub_distance(self):
        dis = (ultrasonic.getDistance())
        # print(dis)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasound'
        msg.radiation_type = 0
        msg.min_range = 0.0
        msg.max_range = 300.0
        msg.range = dis
        self.publisher_.publish(msg)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = pub_distance("pub_distance")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()