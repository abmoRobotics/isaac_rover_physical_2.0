#!/usr/bin/env python3
from pickle import FALSE
from unittest import case
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Joy
import math
import cmath

max_speed = 1
new_max = 0

global l, r, u, d

l = 0
r = 0
u = 0 
d = 0


class joy_listener(Node):

        

    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy/joy', self.listener_callback,1000)


    def listener_callback(self, msg):
        global l, r, u, d

        LR_stick = msg.axes[0] # Moving the robot left and right
        UD_stick = msg.axes[1] # Moving the robot forward and backward

        LR_cross = msg.axes[6]  # Left = 0-50 % and right = 0-75%
        UP_cross = msg.axes[7]  # Down = 0-25% UP = 0-100% 

        button_A = msg.buttons[1] # Make the robots wheels set to turn around itself
        button_B = msg.buttons[2] # Make the robot wheels set to go forward and backward
        
        button_Y = msg.buttons[3] # Breaks the automatic mode
        button_X = msg.buttons[0] # Starts the automatic mode

        butten_LT = msg.axes[2] - 1# Increse or decrese the speed of the robot forward
        butten_RT = msg.axes[5] # Increse or decrese the speed of the robot backward

        #self.get_logger().info("LR_cross: -1    ==  " + str(LR_cross))

        
        u=-LR_stick
        v=UD_stick
        self.get_logger().info("LR_STICK: " + str(u) + " UD_STICK: " + str(v))
        x=1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)+2*u*math.sqrt(2)))-1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)-2*u*math.sqrt(2)))

        y=1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)+2*v*math.sqrt(2)))-1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)-2*v*math.sqrt(2)))
        self.get_logger().info("x: " + str(x) + " y: " + str(v) )
        if LR_cross == -1.0 :
            r = 1
            l = 0 
            u = 0
            d = 0
            self.get_logger().info("Mode" + str(l))

        elif LR_cross == 1.0 :
            l = 1
            r = 0 
            u = 0
            d = 0
            self.get_logger().info("Mode" + str(r))

        elif UP_cross == -1.0 :
            d = 1
            l = 0 
            u = 0
            r = 0
            self.get_logger().info("Mode" + str(u))
        
        elif UP_cross == 1.0 :
            u = 1
            l = 0 
            r = 0
            d = 0
            self.get_logger().info("Mode" + str(d))

        if l == 1:
            new_max = ((0.75 * max_speed)*butten_LT)/-2
            self.get_logger().info("Mode" + str(l) + " Speed: " + str(new_max))

        elif r == 1:
            new_max = ((0.50 * max_speed)*butten_LT)/-2
            self.get_logger().info("Mode" + str(r) + " Speed: " + str(new_max))

        elif u == 1:
            new_max = ((1.00 * max_speed)*butten_LT)/-2
            self.get_logger().info("Mode" + str(u) + " Speed: " + str(new_max))
        
        elif d == 1:
            new_max = ((0.25 * max_speed)*butten_LT)/-2
            self.get_logger().info("Mode" + str(d) + " Speed: " + str(new_max))
            
    

        
        #st_angle, mo_speed = scripts.kinematicsCPU(LR_stick, butten_LT)

        #self.get_logger().info("Steering angle: " + str(st_angle) + "Motor speed: " + str(mo_speed))


def main(args=None):
    rclpy.init(args=args)

    joy_listener1 = joy_listener()

    joy_listener1
    rclpy.spin(joy_listener1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_listener1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

