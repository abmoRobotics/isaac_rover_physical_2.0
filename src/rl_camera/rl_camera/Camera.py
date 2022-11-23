#!/usr/bin/env python
#extract the highmap
#ones every 0.25s the camarea node should get the highmap data and send it to a neural network node
#publish it to a topic

import rclpy
from rclpy.node import Node
from camera_utils import ZedCamera
from sensor_msgs.msg import PointCloud2

class Camera_newnode(Node):
    """Convert Motor Commands"""

    def __init__(self):
        
        """Init Node."""
        self.node_name = 'Camera_newnode'
        super().__init__(self.node_name)

        self.pub = self.create_publisher( #Heightmap publisher
                PointCloud2,
                'PointCloud2',
                1)
        
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.callback)

        """Init Camera."""
        self.camera = ZedCamera()
        
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))
       

 

    def callback(self):
        msg = PointCloud2()
        self.get_logger().info("inside callback")
           
        #start = time.perf_counter()  #returns the float value of time in s
        heightData  = self.camera.callback() ### 0.14s - 0.294s
        msg.data  = heightData
        self.get_logger().info(str(heightData))
        #end = time.perf_counter() - start
            


def main(args=None):
    rclpy.init(args=args)
    CameraNode = Camera_newnode()
    rclpy.spin(CameraNode)
    CameraNode.destroy_node()
    rclpy.shutdown()
    



if __name__ == '__main__':
    main()