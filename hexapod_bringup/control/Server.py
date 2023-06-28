# -*- coding: utf-8 -*-
import io
import time
import fcntl
import socket
import struct
# from picamera2 import Picamera2,Preview
# from picamera2.encoders import JpegEncoder
# from picamera2.outputs import FileOutput
# from picamera2.encoders import Quality
from threading import Condition
import threading
from Led import *
from Servo import *
from Thread import *
from Buzzer import *
from Control import *
from ADC import *
from Ultrasonic import *
from Command import COMMAND as cmd

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class Server:
    def __init__(self):
        self.tcp_flag=False
        self.led=Led()
        self.adc=ADC()
        self.servo=Servo()
        self.buzzer=Buzzer()
        self.control=Control()
        self.sonic=Ultrasonic()
        self.control.Thread_conditiona.start()
    def get_interface_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(),
                                            0x8915,
                                            struct.pack('256s',b'wlan0'[:15])
                                            )[20:24])
    def turn_on_server(self):
        #ip adress
        HOST=self.get_interface_ip()
        #Port 8002 for video transmission
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEPORT,1)
        self.server_socket.bind((HOST, 8002))              
        self.server_socket.listen(1)
        
        #Port 5002 is used for instruction sending and receiving
        self.server_socket1 = socket.socket()
        self.server_socket1.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEPORT,1)
        self.server_socket1.bind((HOST, 5002))
        self.server_socket1.listen(1)
        print('Server address: '+HOST)
        
    def turn_off_server(self):
        try:
            self.connection.close()
            self.connection1.close()
        except :
            print ('\n'+"No client connection")
    
    def reset_server(self):
        self.turn_off_server()
        self.turn_on_server()
        self.video=threading.Thread(target=self.transmission_video)
        self.instruction=threading.Thread(target=self.receive_instruction)
        self.video.start()
        self.instruction.start()
    def send_data(self,connect,data):
        try:
            connect.send(data.encode('utf-8'))
            #print("send",data)
        except Exception as e:
            print(e)
    # def transmission_video(self):
    #     try:
    #         self.connection,self.client_address = self.server_socket.accept()
    #         self.connection=self.connection.makefile('wb')
    #     except:
    #         pass
    #     self.server_socket.close()
    #     print ("socket video connected ... ")
    #     camera = Picamera2()
    #     camera.configure(camera.create_video_configuration(main={"size": (400, 300)}))
    #     output = StreamingOutput()
    #     encoder = JpegEncoder(q=90)
    #     camera.start_recording(encoder, FileOutput(output),quality=Quality.VERY_HIGH) 
    #     while True:
    #         with output.condition:
    #             output.condition.wait()
    #             frame = output.frame
    #         try:                
    #             lenFrame = len(output.frame) 
    #             #print("output .length:",lenFrame)
    #             lengthBin = struct.pack('<I', lenFrame)
    #             self.connection.write(lengthBin)
    #             self.connection.write(frame)
    #         except Exception as e:
    #             camera.stop_recording()
    #             camera.close()
    #             print ("End transmit ... " )
    #             break

    def receive_instruction(self):
        try:
            self.connection1,self.client_address1 = self.server_socket1.accept()
            print ("Client connection successful !")
        except:
            print ("Client connect failed")
        self.server_socket1.close()
        
        while True:
            try:
                allData=self.connection1.recv(1024).decode('utf-8')
            except:
                if self.tcp_flag:
                    self.reset_server()
                    break
                else:
                    break
            if allData=="" and self.tcp_flag:
                self.reset_server()
                break
            else:
                cmdArray=allData.split('\n')
                print(cmdArray)
                if cmdArray[-1] !="":
                    cmdArray==cmdArray[:-1]
            for oneCmd in cmdArray:
                data=oneCmd.split("#")
                if data==None or data[0]=='':
                    continue
                elif cmd.CMD_BUZZER in data:
                    self.buzzer.run(data[1])
                elif cmd.CMD_POWER in data:
                    try:
                        batteryVoltage=self.adc.batteryPower()
                        command=cmd.CMD_POWER+"#"+str(batteryVoltage[0])+"#"+str(batteryVoltage[1])+"\n"
                        #print(command)
                        self.send_data(self.connection1,command)
                        if batteryVoltage[0] < 5.5 or batteryVoltage[1]<6:
                         for i in range(3):
                            self.buzzer.run("1")
                            time.sleep(0.15)
                            self.buzzer.run("0")
                            time.sleep(0.1)
                    except:
                        pass
                elif cmd.CMD_LED in data:
                    try:
                        stop_thread(thread_led)
                    except:
                        pass
                    thread_led=threading.Thread(target=self.led.light,args=(data,))
                    thread_led.start()   
                elif cmd.CMD_LED_MOD in data:
                    try:
                        stop_thread(thread_led)
                        #print("stop,yes")
                    except:
                        #print("stop,no")
                        pass
                    thread_led=threading.Thread(target=self.led.light,args=(data,))
                    thread_led.start()
                elif cmd.CMD_SONIC in data:
                    command=cmd.CMD_SONIC+"#"+str(self.sonic.getDistance())+"\n"
                    self.send_data(self.connection1,command)
                elif cmd.CMD_HEAD in data:
                    if len(data)==3:
                        self.servo.setServoAngle(int(data[1]),int(data[2]))
                elif cmd.CMD_CAMERA in data:
                    if len(data)==3:
                        x=self.control.restriction(int(data[1]),50,180)
                        y=self.control.restriction(int(data[2]),0,180)
                        self.servo.setServoAngle(0,x)
                        self.servo.setServoAngle(1,y)
                elif cmd.CMD_RELAX in data:
                    #print(data)
                    if self.control.relax_flag==False:
                        self.control.relax(True)
                        self.control.relax_flag=True
                    else:
                        self.control.relax(False)
                        self.control.relax_flag=False
                elif cmd.CMD_SERVOPOWER in data:
                    if data[1]=="0":
                        GPIO.output(self.control.GPIO_4,True)
                    else:
                        GPIO.output(self.control.GPIO_4,False)
                    
                else:
                    self.control.order=data
                    self.control.timeout=time.time()
        try:
            stop_thread(thread_led)
        except:
            pass
        try:
            stop_thread(thread_sonic)
        except:
            pass
        print("close_recv")

if __name__ == '__main__':
    pass
    
