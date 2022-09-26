#!/usr/bin/env python3
from pickle import FALSE
from unittest import case
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Joy
import numpy as np
import math
import cmath

max_speed = 1
new_max = 0
new_min=0
linear_vel=0
ang=0

global mode, autom
mode=0
autom=1

class joy_listener(Node):

        

    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy/joy', self.listener_callback,1000)


    def listener_callback(self, msg):
        global autom,mode,ang

        #IMPORTANT: mode button changes the joystick's mapping. Make sure the little led next to it is off 

        LR_stick = msg.axes[0] # Moving the robot left and right  0
        UD_stick = msg.axes[1] # Moving the robot forward and backward  1

        LR_cross = msg.axes[6]  # Left = 0-50 % and right = 0-75%  6
        UP_cross = msg.axes[7]  # Down = 0-25% UP = 0-100%   7

        button_A = msg.buttons[0] # Make the robots wheels set to turn around itself
        button_B = msg.buttons[1] # Make the robot wheels set to go forward and backward
        
        button_Y = msg.buttons[3] # Breaks the automatic mode
        button_X = msg.buttons[2] # Starts the automatic mode

        butten_LT = msg.axes[2] - 1# Increse or decrese the speed of the robot forward
        butten_RT = msg.axes[5]-1 # Increse or decrese the speed of the robot backward

        #self.get_logger().info("LR_cross: -1    ==  " + str(LR_cross))
        if button_Y==1:
            autom=0
        elif button_X==1:
            autom=1 
        if LR_cross == -1.0 :
            mode=1

        elif LR_cross == 1.0 :
            mode=2

        elif UP_cross == -1.0 :
            mode=4
        
        elif UP_cross == 1.0 :
            mode=3
        #self.get_logger().info("auto: " + str(autom) + " x: " + str(button_X)+ " y: " + str(button_Y))
        #self.get_logger().info("Mode" + str(LR_cross)+ str(UP_cross))

        u=-LR_stick
        v=UD_stick
        self.get_logger().info("LR_STICK: " + str(u) + " UD_STICK: " + str(v))
        x=1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)+2*u*math.sqrt(2)))-1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)-2*u*math.sqrt(2)))

        y=1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)+2*v*math.sqrt(2)))-1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)-2*v*math.sqrt(2)))
        
        r=math.sqrt(math.pow(x,2)+math.pow(y,2))

        if 0.0<=u<=1.0 and abs(v)==0.0:
            ang=0
        elif 0.0<=u<=1.0 and 0.0<=v<=1.0:
            ang=np.arctan(v/u)
        elif 0.0<=v<=1.0 and abs(u)==0.0:
            ang=cmath.pi/2
        elif -1.0<=u<=-0.0 and 0.0<=v<=1.0:
            ang=np.arctan(v/u)+cmath.pi
        elif -1.0<=u<=-0.0 and abs(v)==0.0:
            ang=cmath.pi
        
        elif -1.0<=v<=-0.0 and abs(u)==0.0:
            ang=cmath.pi*(-1/2)
        
        

    
        self.get_logger().info("x: " + str(x) + " y: " + str(v) + " r: " + str(r)+  " a: " + str(ang))
        
        if autom==0:

            if mode == 2:
                new_max = ((0.75 * max_speed)*butten_LT)/-2
                new_min = ((0.75 * max_speed)*butten_RT)/2
                linear_vel=new_max + new_min
                self.get_logger().info("Mode" + str(mode) + " Speed LR: " + str(new_max)+" Speed RT: " + str(new_min)+" Linear v: " + str(linear_vel))

            elif mode == 1:
                new_max = ((0.50 * max_speed)*butten_LT)/-2
                new_min = ((0.50 * max_speed)*butten_RT)/2
                linear_vel=new_max + new_min
                self.get_logger().info("Mode" + str(mode) + " Speed: " + str(new_max)+" Speed RT: " + str(new_min))

            elif mode == 3:
                new_max = ((1.00 * max_speed)*butten_LT)/-2
                new_min = ((1.00 * max_speed)*butten_RT)/2
                linear_vel=new_max + new_min
                self.get_logger().info("Mode" + str(mode) + " Speed: " + str(new_max)+" Speed RT: " + str(new_min))
            
            elif mode == 4:
                new_max = ((0.25 * max_speed)*butten_LT)/-2
                new_min = ((0.25 * max_speed)*butten_RT)/2
                linear_vel=new_max + new_min
                self.get_logger().info("Mode" + str(mode) + " Speed: " + str(new_max)+" Speed RT: " + str(new_min))
                
    

        
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

