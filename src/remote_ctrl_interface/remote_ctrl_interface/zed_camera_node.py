#!/usr/bin/env python
from email.mime import image
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32
import numpy as np
import sys
import pyzed.sl as sl
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import socket
import cv2
import base64
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
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(
                CompressedImage,
                '/zed_camera/image',
                qos_profile=qos_profile)
        #self.timer = self.create_timer(1, self.callback)
        self.IP = '169.254.148.155'
        self.PORT = 8080
        self.socket = socket.socket()
        self.socket.connect((self.IP, self.PORT))
        self.encoding = [int(cv2.IMWRITE_JPEG_QUALITY),90]
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))
      
        
    def callback(self):
        bridge = CvBridge()
        zed = sl.Camera()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sdk_verbose = False
        # Set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # set resolution
        init_params.camera_fps = 15 
        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Get camera information (ZED serial number)
        zed_serial = zed.get_camera_information().serial_number
        self.get_logger().info("Camera detected. Serial number: {0}".format(zed_serial))

        image_sl = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        while True:
            # Grab an image, a RuntimeParameters object must be given to grab()
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns ERROR_CODE.SUCCESS
                try:
                    zed.retrieve_image(image_sl, sl.VIEW.LEFT) # Get the left image
                    timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)  # Get the image timestamp
                    self.get_logger().info("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image_sl.get_width(), image_sl.get_height(), timestamp.get_milliseconds()))
                    image_arr = image_sl.get_data()
                    res, encoded_img = cv2.imencode('.jpg', image_arr, self.encoding)
                    data = np.array(encoded_img)
                    data_string = base64.b64encode(data)
                    length = str(len(data_string))
                    self.socket.sendall(length.encode('utf-8').ljust(64))
                    self.socket.send(data_string)
                    #image_message = bridge.cv2_to_compressed_imgmsg(image_arr)
                    #self.get_logger().info("Converted to Img")
                    #self.pub.publish(image_message)
                except:
                    self.get_logger().info("An error occured")


        # Close the camera
        zed.close()


def main(args=None):
    rclpy.init(args=args)

    try:
        zed_camera_node = Zed_camera_node()
        try:
            zed_camera_node.callback()
        except:
            print("An error occured")
        finally:
            zed_camera_node.socket.close()
            zed_camera_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()