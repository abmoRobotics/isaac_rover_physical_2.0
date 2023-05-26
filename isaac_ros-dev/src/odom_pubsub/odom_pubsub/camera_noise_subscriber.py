#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge


from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image 

# Create a ROS2 node

class Camera_noise(Node):

    def __init__(self):
        super().__init__('camera_noise')
        self.rgb_subscription = self.create_subscription(
            Image,
            '/rgb_right',
            self.rgb_callback,
            10)
        
        # self.depth_subscription = self.create_subscription(
            # Image,
            # '/depth_right',
            # self.depth_callback,
            # 10)
        self.target_frame = "map"
        self.br = CvBridge()
        
        # self.subscription  # prevent unused variable warning

# Create a callback function to print the subscribed output
 
    # def timer_callback(self):
    #     img = Image()
    #     img_blur = self.br.imgmsg_to_cv2(img)
    #     img_blur = cv2.blur(img_blur, (21,21))
    #     self.get_logger().info('Publishing an image')


    def rgb_callback(self, msg):
        image_blured = Image()
        image_blured.header.stamp = self.get_clock().now().to_msg()
        image_blured.header.frame_id = self.target_frame
        
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.blur(current_frame, (21,21))
        cv2.imshow("test", current_frame)
        cv2.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)

    camera_noise = Camera_noise()
   
   # Spin the node so the callback function is called.
    rclpy.spin(camera_noise)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_noise.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()