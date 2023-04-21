#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

# Create a ROS2 node

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.callback,
            10)
        
        self.subscription  # prevent unused variable warning

# Create a callback function to print the subscribed output
 
    def callback(self, msg):

        # angular_velocity
        x_ang = msg.angular_velocity.x
        y_ang = msg.angular_velocity.y
        z_ang = msg.angular_velocity.z


        # linear_velocity
        x_lin = msg.linear_acceleration.x
        y_lin = msg.linear_acceleration.y
        z_lin = msg.linear_acceleration.z

        # Orientation
        x_orient = msg.orientation.x
        y_orient = msg.orientation.y
        z_orient = msg.orientation.z
        w = msg.orientation.w

        print('angular_velocity x:',x_ang , 'angular_velocity y:', y_ang , 'angular_velocity z:', z_ang)
        print('linear_acceleration x:',x_lin , 'linear_acceleration y:', y_lin , 'linear_acceleration z:', z_lin)
        print('orient x:',x_orient , 'orient y:', y_orient , 'orient z:', z_orient, 'w:', w)
        print(' ')

        
    def listener_callback(self, msg):
        self.get_logger().info(msg.pose.pose)

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()










# def callback(msg):
#     print(msg.pose.pose)

# rclpy.init_node('check_odometry')
# odom_sub = rclpy.Subscriber('/odom', Odometry)

