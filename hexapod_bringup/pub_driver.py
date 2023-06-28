#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Trigger
import tf_transformations
import sys
import time
sys.path.append('/home/spider/hexapod_ws/src/hexapod/hexapod/control')
from IMU import *
from ADC import *
from Control import *
from Buzzer import *
from Led import *

imu = IMU()    
# adc=ADC()   
c=Control()
led=Led()

battery_check = 0
time.sleep(1)
class pub_driver(Node):    
    def __init__(self, name):
        super().__init__(name)
        self.buzzer=Buzzer() 
        
        self.publisher_imu = self.create_publisher(Imu,'Imu_data', 0)
        self.sub = self.create_subscription(Joy,'joy', self.joy_callback,0)
        # self.publisher_bat = self.create_publisher(Float32MultiArray,'battery_data', 0)
        self.sub_vel = self.create_subscription(Twist,'/spider/cmd_vel', self.vel_callback,0)
        # self.sub_pose = self.create_subscription(Pose,'pose', self.pose_callback,0)
        self.sub_led = self.create_subscription(Float32MultiArray,'led_data', self.led_callback,0)
        
        self.timer1 = self.create_timer(0.001, self.pub_imu) 
        # self.timer2 = self.create_timer(0.1, self.pub_battery) 
        self.button_pressed = 0
        
    def joy_callback(self, data=[]):
        global joy_data
        joy_data = data
        
    def led_callback(self, data):
        led_data = data
        if len(led_data.data) > 0:
            led.colorWipe(led.strip, Color(int(led_data.data[0]), int(led_data.data[1]), int(led_data.data[2]))) 
        
    def pub_imu(self): # publish imu data
        time.sleep(0.000001)
        q0,q1,q2,q3=imu.imuUpdate() # get the imu data
        self.imu_data = Imu()
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.imu_data.header.frame_id = 'IMU'   
        self.imu_data.orientation.x = q0 
        self.imu_data.orientation.y = q1 
        self.imu_data.orientation.z = q2 
        self.imu_data.orientation.w = q3
        self.publisher_imu.publish(self.imu_data) # publsih

    # def pub_battery(self): # publish battery voltage
    #     global battery_check
    #     batteryVoltage = adc.batteryPower() # get the battery voltage
    #     if battery_check ==0 and (batteryVoltage[0] < 5.5 or batteryVoltage[1]< 6.0): # Buzzer will run when voltage is too low
    #         for i in range(1):
    #             self.buzzer.run("1")
    #             time.sleep(0.05)
    #             self.buzzer.run("0")
    #             time.sleep(0.05)
    #         battery_check = 1
    #     self.data = Float32MultiArray()
    #     self.data.data.append(batteryVoltage[0])
    #     self.data.data.append(batteryVoltage[1])
    #     self.publisher_bat.publish(self.data) # publsih
        
    def vel_callback(self, data): #cmd_vel callback
        twist = data 
        self.move(twist)
        
    def move(self, twist): #整體移動or轉彎
        if joy_data.buttons[7]==0 and self.button_pressed == 0:
            x = twist.linear.x
            y = twist.linear.y
            omega_z = twist.angular.z * 10 #測試x, y大小對速度的影響
            x = x*35/1
            y = y*35/1
            vel = 10        
            self.data=['CMD_MOVE', '1', -y, x, vel, -omega_z] # '1'= 3 leg run , '2' = 1 leg run at the same time , '10' = speed
            if self.data != ['CMD_MOVE', '1', 0.0, 0.0, 10, 0.0]:
                c.run(self.data) # run the spider.
        self.relax()
    
    def relax(self):
        if joy_data.buttons[5]==1 and self.button_pressed == 0:
            c.relax(True)
            self.button_pressed = 1
        elif joy_data.buttons[4]==1 and self.button_pressed == 1:
            c.relax(False)
            self.button_pressed = 2
        elif self.button_pressed == 2:
            self.button_pressed = 0
            
    # def pose_callback(self, data):
    #     pose = data
    #     x = pose.position.x*70
    #     y = pose.position.y*70
    #     z = pose.position.z*90
    #     if (joy_data.buttons[6]==1 ): 
    #         c.posittion(x, y, z) #spider 身體移動
            
    #     elif (joy_data.buttons[7]==1 ): #spider姿態移動
    #         roll, pitch, yaw = tf_transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])      
    #         point=c.postureBalance(roll*5, pitch*5, 0)
    #         c.coordinateTransformation(point)
    #         c.setLegAngle()
            


        


def main(args=None):
    rclpy.init(args=args)
    node = pub_driver("pub_driver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()