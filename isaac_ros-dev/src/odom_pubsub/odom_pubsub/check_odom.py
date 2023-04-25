#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

# Create a ROS2 node

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback,
            10)
        
        self.subscription  # prevent unused variable warning

# Create a callback function to print the subscribed output
 
    def callback(self, msg):

        # Pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Orientation
        x_orient = msg.pose.pose.orientation.x
        y_orient = msg.pose.pose.orientation.y
        z_orient = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        print('pose x:',x , ' y:', y , ' z:', z)
        
        print('orient x:',x_orient , ' y:', y_orient , ' z:', z_orient, 'w:', w)
        print(' ')

        
    def listener_callback(self, msg):
        self.get_logger().info(msg.pose.pose)

def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()










# def callback(msg):
#     print(msg.pose.pose)

# rclpy.init_node('check_odometry')
# odom_sub = rclpy.Subscriber('/odom', Odometry)

