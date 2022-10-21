#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge
import cv2
from rover_msgs.msg import GoalPoint2d
import socket
import base64


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
                GoalPoint2d,
                'GoalPoint2d',
                1)
        self.IP = '169.254.148.155'
        self.PORT = 8080
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.IP, self.PORT))
        self.socket.listen(1)
        self.conn, self.addr = self.socket.accept()
        self.img_width = -1
        self.img_height = -1        
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))


    def recvall(self, sock, count):
        # http://stupidpythonideas.blogspot.com/2013/05/sockets-are-byte-streams-not-message.html
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def rec_images(self):
        while True:
            length = self.recvall(self.conn, 64)
            length_dec = length.decode('utf-8')
            stringData = self.recvall(self.conn, int(length_dec))
            data = np.frombuffer(base64.b64decode(stringData), np.uint8)
            decimg = cv2.imdecode(data, 1)
            self.show_img(decimg)

      
        
    def show_img(self, recived_img):
        global goal_selected, goal_x, goal_y
        try:
            self.img_width = recived_img.shape[1]
            self.img_height = recived_img.shape[0]
            cv2.namedWindow("Live feed")
            cv2.setMouseCallback("Live feed", self.mouse_callback)
            cv2.imshow("Live feed", recived_img)
            cv2.waitKey(1)
            if(goal_selected):
                frame = cv2.circle(recived_img, (goal_x, goal_y), radius=10, color=(255, 255, 255), thickness=-1)
                cv2.imshow('Live feed', recived_img)
                cv2.setMouseCallback("Live feed", self.mouse_callback)
            
        except Exception as e: 
           self.get_logger().info('\tERROR: {}'.format(e))

    
    def mouse_callback(self, event, x, y, flags, param):
        global goal_selected, goal_x, goal_y
        if event == cv2.EVENT_LBUTTONUP:
            goal_selected = True
            goal_x = x
            goal_y = y
            msg = GoalPoint2d()
            # normalize
            msg.x = goal_x/self.img_width
            msg.y = goal_y/self.img_height
            
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        live_feed = Live_feed_node()
        live_feed.rec_images()
        try:
            rclpy.spin(live_feed)
        finally:
            live_feed.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()
