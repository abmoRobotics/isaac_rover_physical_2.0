#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from exomy_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32
import numpy as np
import sys
import pyzed.sl as sl
from cv_bridge import CvBridge
#from sensor_msgs_py import point_cloud2
sys.path.append('/home/xavier/ExoMy_Software/exomy/scripts/utils')
sys.path.append('/home/xavier/ros2_numpy')
import ros2_numpy

class Zed_camera_node(Node):
    """Publishes zed images"""

    def __init__(self):
        """Init Node."""

        self.node_name = 'Camera_node'
        super().__init__(self.node_name)
        self.pub = self.create_publisher(
                Image,
                '/zed_camera/image',
                1)

        
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))
      
        
    def callback(self):
        bridge = CvBridge()
        # Create a Camera object
        zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sdk_verbose = False
        # Set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080  # set resolution
        init_params.camera_fps = 15 

        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Get camera information (ZED serial number)
        zed_serial = zed.get_camera_information().serial_number
        print("Camera detected. Serial number: {0}".format(zed_serial))

        image_sl = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        while True:
            # Grab an image, a RuntimeParameters object must be given to grab()
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns ERROR_CODE.SUCCESS
                zed.retrieve_image(image_sl, sl.VIEW.LEFT) # Get the left image
                timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)  # Get the image timestamp
                print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image_sl.get_width(), image_sl.get_height(), timestamp.get_milliseconds()))
                image_arr = image_sl.get_data()
                image_message = bridge.cv2_to_imgmsg(image_arr, encoding="passthrough")
                # TODO: publish image
                self.pub.publish(image_message)


        # Close the camera
        zed.close()


def main(args=None):
    rclpy.init(args=args)

    try:
        Zed_camera_node = Zed_camera_node()
        Zed_camera_node.callback()
        try:
            rclpy.spin(Zed_camera_node)
        finally:
            Zed_camera_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()