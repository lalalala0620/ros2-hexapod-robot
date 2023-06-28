import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库
import tf_transformations
from geometry_msgs.msg import Twist, Pose
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
can_go = True

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(TFMessage, '/tf_april', self.listener_callback, 10)     
        self.publisher_twist = self.create_publisher(Twist, 'cmd_vel_tag', 10)
        self.sub_status = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav2_callback, 10) 
        self.publisher_led = self.create_publisher(Float32MultiArray,'led_data', 0)
        self.sub_joy = self.create_subscription(Joy,'joy', self.joy_callback,0)
    
    def joy_callback(self, data):
        global joy_data
        joy_data = data
        
    def nav2_callback(self, data):
        global can_go 
        led_data = Float32MultiArray()
        led_data.data = []
        # print(data.status_list)
        # print((data.status_list[len(data.status_list)-1].status))
        if data.status_list[len(data.status_list)-1].status == 2:
            led_data.data.append(0)
            led_data.data.append(0)
            led_data.data.append(255)
            self.publisher_led.publish(led_data)
        elif data.status_list[len(data.status_list)-1].status == 4 :
            led_data.data.append(0)
            led_data.data.append(255)
            led_data.data.append(0)
            self.publisher_led.publish(led_data)
            can_go = True
        else:
            led_data.data.append(0)
            led_data.data.append(0)
            led_data.data.append(0)
            self.publisher_led.publish(led_data)
        
        
        
    def listener_callback(self, data):
        print(can_go)
        if data.transforms != []:
            # print((data.transforms[0].transform.translation.x))
            translation = data.transforms[0].transform.translation
            rotation = data.transforms[0].transform.rotation
            euler_angle = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            # print(translation, rotation)
            if can_go == True:
                print(can_go)
                self.pub_cmdvel(translation, euler_angle)
            
        # for transform in data.transforms:
        #     print("1")
        #     parent_frame_id = transform.header.frame_id
        #     child_frame_id = transform.child_frame_id
        #     translation = transform.transform.translation
        #     rotation = transform.transform.rotation
        #     euler_angle = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        #     # print(np.rad2deg(euler_angle[1]))
        #     if can_go == True:
        #     # print(euler_angle)
        #         print(can_go)
        #         self.pub_cmdvel(translation, euler_angle)
            
            
    def pub_cmdvel(self, position, euler_angle, ):
        twist = Twist()
        
        twist.linear.x = 1/0.5 * (-0.4 - (-position.z))
        twist.linear.y = 1 * (0 - position.x)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (0-euler_angle[1]) * 2/3.14                
        self.publisher_twist.publish(twist)




def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("apriltag_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口


if __name__ == "__main__":
    main()