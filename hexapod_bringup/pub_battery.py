#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
sys.path.append('/home/spider/hexapod_ws/src/hexapod/hexapod/control')
import smbus
import time
from ADCDevice import *

class ADC:
    def __init__(self):
        self.adcFlag = None
        self.adc = ADCDevice()
        if(self.adc.detectI2C(0x4f)): # Detect the pcf8591.
            self.adcFlag = False
            self.adc = PCF8591()
        elif(self.adc.detectI2C(0x48)): # Detect the ads7830
            self.adcFlag = True
            self.adc = ADS7830()
        else:
            print("No correct I2C address found, \n"
            "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
            "Program Exit. \n");
            exit(-1)
    def batteryValue(self,chn):
        return self.adc.analogRead(chn)
        
    def batteryPower(self): 
        if self.adcFlag == True:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(4)
        else:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(1)
            
        battery1=round(val0/255*5*3,2)
        battery2=round(val1/255*5*3,2)
        #print(str(self.adc.address)+" "+str(val0)+" "+str(val1))
        return battery1,battery2
adc=ADC()    
class pub_battery(Node):    
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Float32MultiArray,'battery_data', 0)
        self.timer = self.create_timer(0.1, self.pub_battery) 
        
    def pub_battery(self):
        power=adc.batteryPower()
        data = Float32MultiArray()
        print(power[0])
        data.data.append(power[0])
        data.data.append(power[1])
        print(data.data)
        self.publisher_.publish(data)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = pub_battery("pub_battery")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()