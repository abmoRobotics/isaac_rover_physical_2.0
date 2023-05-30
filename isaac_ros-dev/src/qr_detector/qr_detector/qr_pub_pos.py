#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image 


from pyzbar.pyzbar import decode

class QRDetector:
    def __init__(self):
        # Camera matrix parameters from sensor_msgs/CameraInfo.msg
        self.FX = 1465.99853515625
        self.FY = 1468.26416015625
        self.PPX = 640.0
        self.PPY = 360.0

        self.fPixels = self.FX

        # distance between cameras in meters
        self.b = 0.004

        self.CamMatrix = np.asarray([[self.FX, 0.0, self.PPX, 0],
                                [0.0, self.FY, self.PPY, 0],
                                [0.0, 0.0, 1.0, 0]])
        
    # Use this function to create a custon camera matrix
    def bringup_camera_matrix(self, x, fy, ppx, ppy):
        '''

        :param fx: focal length x
        :param fy: focal length y
        :param ppx: principal point
        :param ppy: principal point
        :return:
        '''
        camera_matrix = np.asarray([[fx, 0.0, ppx, 0],
                                    [0.0, fy, ppy, 0],
                                    [0.0, 0.0, 1.0, 0]])

        return camera_matrix

    # If depth map from ros2 topic is not provided use this
    # TODO: Provide a disparity map

    def get_coordinate(self, coord_2d, disparity_map):
        '''
        :param coord_2d: coord x,y in pixel coords
        :param disparity_map: the disparity map
        :return:
        '''
        depth_map = (b * fPixels) / disparity_map
        matrix = CamMatrix
        u = int(coord_2d[0])
        v = int(coord_2d[1])
        depth = depth_map[v][u]

        X = depth * (u - matrix[0][2]) / (matrix[0][0])
        Y = depth * (v - matrix[1][2]) / (matrix[1][1])
        Z = depth
        return [X, Y, Z]

    def detect(self, image_rgb):
        cv2.imshow("test", image_rgb)
        cv2.waitKey(1)
        for code in decode(image_rgb):
            print ("Found a QR CODE")
            qr_link = code.data.decode("utf-8")
            print(code.data)
            print()

            # b = webbrowser.open(qr_link)


            # Draw BBOX
            bbox_points = code.polygon
            # print(bbox_points)

            # See the how are the points and where they belong to
            points = np.array([code.polygon], np.int32)

            # Drawing the boundy box  the center of it and the position of each point
            cv2.polylines(image_rgb, [points], True, (255, 0, 0), 3)
            cv2.putText(image_rgb, "0", (points[0][0][0], points[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,0), 2)
            cv2.putText(image_rgb, "1", (points[0][1][0], points[0][1][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(image_rgb, "2", (points[0][2][0], points[0][2][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(image_rgb, "3", (points[0][3][0], points[0][3][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            #Getting the center of the bbx

            center_bbx = [abs((points[0][1][0] + points[0][0][0])/2), abs((points[0][2][1] + points[0][1][1])/2)]
            # print(center_bbx)
            cv2.circle(image_rgb, (int(center_bbx[0]), int(center_bbx[1])), 2, (255, 0, 0), 2)

            # Show the image
            print ("Image has been generated ")
            #cv2.imshow("qr", image_rgb)
            #cv2.waitKey(1)
            print ("DOne!!! ")
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break


class ColorAndDepth_Subscriber(Node):
    def __init__(self):
        super().__init__('camera_noise')
        self.rgb_subscription = self.create_subscription(
            Image,
            '/rgb_right',
            self.rgb_callback,
            10)
        
        #self.rgb_subscription = self.create_subscription(
        #    Image,
        #    '/depth_right',
        #    self.rgb_callback,
        #    10)
        
        self.target_frame = "map"
        self.br = CvBridge()

        self.qr_detector = QRDetector()
        
    def rgb_callback(self, msg):
        image_blured = Image()
        image_blured.header.stamp = self.get_clock().now().to_msg()
        image_blured.header.frame_id = self.target_frame
        
        current_frame = self.br.imgmsg_to_cv2(msg)
        print ("Foo1")
        self.qr_detector.detect(current_frame)
        print ("Foo2")
        #current_frame = cv2.blur(current_frame, (21,21))
        #cv2.imshow("test", current_frame)
        #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ColorAndDepth_Subscriber()
   
   # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
