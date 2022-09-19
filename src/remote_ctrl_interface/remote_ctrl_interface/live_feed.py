#!/usr/bin/env python
from typing_extensions import Self
import rclpy
from rclpy.node import Node
from exomy_msgs.msg import CameraData
import numpy as np
import time
import sys
import message_filters
sys.path.append('/home/xavier/isaac_rover_physical/exomy/scripts/utils')
sys.path.append('/home/xavier/ros2_numpy')
import ros2_numpy
from CameraSys import Cameras
import torch
from cv_bridge import CvBridge
import cv2
from exomy_msgs.msg import GoalPoint2d


goal_selected = False
goal_x = 0
goal_y = 0

class Live_feed_node(Node):
    """Show video live feed"""

    def __init__(self):

        """Init Node."""
        self.node_name = 'Live_feed_node'
        super().__init__(self.node_name)
        self.pub = self.create_publisher( # Goal point publisher
                GoalPoint,
                'GoalPoint',
                1)

        self.cam1Sub = message_filters.Subscriber(self, CameraData, "/cam_1/depth/color/points")
        #self.cam2Sub = message_filters.Subscriber(self, CameraData, "/cam_1/depth/color/points")

        queue_size = 1
        self.ts = message_filters.ApproximateTimeSynchronizer([self.cam1Sub, self.cam2Sub], queue_size, 0.8)
        self.ts.registerCallback(self.callback)


        """Init Camera."""
        self.camera = Cameras()
        
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))
      
        
       

    def callback(self, data_cam1, data_cam2):
        global goal_selected, goal_x, goal_y
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(data_cam1, desired_encoding='passthrough')
            if(goal_selected):
                frame = cv2.circle(frame, (goal_x, goal_y), radius=2, color=(0, 0, 255), thickness=-1)
                cv2.imshow('Live feed', frame)
                cv2.setMouseCallback("Live feed", self.mouse_callback)
            

        except Exception as e: 
           self.get_logger().info('\tERROR: {}'.format(e))

    
    def mouse_callback(event, x, y, flags, param):
        global goal_selected, goal_x, goal_y
        if event == cv2.EVENT_LBUTTONDOWN:
            goal_selected = True
            goal_x = x
            goal_y = y
            print("Clicked: x = " + str(x) + " | y = " + str(y))
            msg = GoalPoint2d()
            msg.x_pos = goal_x
            msg.y_pos = goal_y
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        Live_feed_node = Live_feed_node()
        try:
            rclpy.spin(Live_feed_node)
        finally:
            Live_feed_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()