#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge


from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image 

# Create a ROS2 node

class Camera_noise_pub(Node):

    def __init__(self, dt = 0.1):
        super().__init__('camera_noise_publish')

        self.rgb_publish = self.create_publisher(
                Image,
                'rgb_right',
                10
            )
        
        self.timer = self.create_timer(dt, self.timer_callback)
        
        self.br = CvBridge()
        self.current_frame = self.br.imgmsg_to_cv2(msg)
        self.current_frame = cv2.blur(current_frame, (21,21))

        
# Create a callback function to print the subscribed output
 
    # def timer_callback(self, blur_img):



    #     img = Image()
    #     #TODO: Compleate the publish info ros2 interface show sensor_msgs/msg/Image 
    #     img.header.stamp = self.get_clock().now().to_msg()
    #     img.header.frame_id = 'base_link'

    #     # Blur
        
    #     self.get_logger().info('Publishing an image')
    #     self.rgb_publish.publish(current_frame)


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        img = self.current_frame
            
        
        self.rgb_publish.publish(self.br.cv2_to_imgmsg(img))
    
        # Display the message on the console
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)

    camera_noise_pub = Camera_noise_pub()
   
   # Spin the node so the callback function is called.
    rclpy.spin(camera_noise_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_noise_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()