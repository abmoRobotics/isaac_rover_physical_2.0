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
import importlib.util

#It makes possible to see the angles and velocities in each robot wheel
spec=importlib.util.spec_from_file_location("kinematicsCPU","/home/xavier/Documents/isaac_rover_physical_2.0/src/controller/scripts/kinematicsCPU.py")
foo=importlib.util.module_from_spec(spec)
spec.loader.exec_module(foo)

#setup variables

max_speed = 1
global mode, autom,rad,ba,new_max,new_min,linear_vel,ang,ang_vel
mode=0
autom=1
rad=0
ba=0
new_max = 0
new_min=0
linear_vel=0
ang=0
ang_vel=0


class joy_listener(Node):

        

    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy/joy', self.listener_callback,1000)


    def listener_callback(self, msg):
        global autom,mode,ang,linear_vel,ang_vel,rad,ba,new_max,new_min,rot_speed

        #IMPORTANT: mode button changes the joystick's mapping. Make sure the little led next to it is off 

        LR_stick = msg.axes[0] # Moving the robot left and right  
        UD_stick = msg.axes[1] # Moving the robot forward and backward  

        #Maximum working speed related to the maximum motor speed
        LR_cross = msg.axes[6]  # Left = 0 to +/-50 % right = 0 to +/-75%  
        UP_cross = msg.axes[7]  # Down = 0 to +/-25% UP = 0 to +/-100%   

        button_A = msg.buttons[0] # Make the robot's wheels set to turn around itself
        button_B = msg.buttons[1] # Make the robot wheels set to drive to any direction
        
        button_Y = msg.buttons[3] # Breaks the automatic mode
        button_X = msg.buttons[2] # Starts the automatic mode

        butten_LT = msg.axes[2] - 1 # Increse or decrese the speed of the robot to move either forward or backward
                                    #In turning mode, makes the robot rotate clockwise
        butten_RT = msg.axes[5]-1 # Only available in turning mode, rotating anticlockwise

        #Control the manual and automatic mode
        if button_Y==1:
            autom=0
        elif button_X==1:
            autom=1

        #Control the maximum speed mode
        if LR_cross == -1.0 :
            mode=1

        elif LR_cross == 1.0 :
            mode=2

        elif UP_cross == -1.0 :
            mode=4
        
        elif UP_cross == 1.0 :
            mode=3

        #Control the driving mode
        elif button_A==1:
            ba=1
        elif button_B==1:
            ba=0    


        #Coordinates in the joystick
        u=-LR_stick
        v=UD_stick
        
        #Transformation to square plane coordinates (not in use)
        x=1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)+2*u*math.sqrt(2)))-1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)-2*u*math.sqrt(2)))

        y=1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)+2*v*math.sqrt(2)))-1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)-2*v*math.sqrt(2)))
        

        #Drive the robot with the joystick
        #All the control goes accordingly the kinematicsCPU code
        if autom==0:
            
            if mode == 2:
                new_max = ((0.75 * max_speed)*butten_LT)/-2
                new_min = ((0.75 * max_speed)*butten_RT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min

            elif mode == 1:
                new_max = ((0.50 * max_speed)*butten_LT)/-2
                new_min = ((0.50 * max_speed)*butten_RT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min

            elif mode == 3:
                new_max = ((1.00 * max_speed)*butten_LT)/-2
                new_min = ((1.00 * max_speed)*butten_RT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min
            
            elif mode == 4:
                new_max = ((0.25 * max_speed)*butten_LT)/-2
                new_min = ((0.25 * max_speed)*butten_RT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min
               
            
            #When the stick is in equilibrium position
            if 0.0==abs(u) and abs(v)==0.0:

                #pressing LT rotates clockwise, pressing RT rotates anticlockwise
                if ba==1:
                    ang_vel=-rot_speed
                    linear_vel=0
                else:
                    ang=0
                    ang_vel=0
                    linear_vel=0

            #Blocks free driving when it is in turning mode
            elif ba==1 and 0.0!=abs(u) and abs(v)!=0.0:
                ang=0
                ang_vel=0
                linear_vel=0

            #Stick in the positive x axis, the robot will turn clockwise
            elif 0.0<=u<=1.0 and abs(v)==0.0:
                ang=0
                ang_vel=-linear_vel
                linear_vel=0
            
            #Stick in the first quadrant, moves forward and turns clockwise
            elif 0.0<u<=1.0 and 0.0<v<=1.0:
                ang=np.arctan(v/u)
                ang_vel=-(linear_vel-(ang*linear_vel/(cmath.pi/2)))
                linear_vel=ang*linear_vel/(cmath.pi/2)
            
            #Stick in the positive y axis, moves forward
            elif 0.0<v<=1.0 and abs(u)==0.0:
                ang=cmath.pi/2
                ang_vel=0
                linear_vel=linear_vel
            
            #Stick in the second quadrant, moves forward and turns anticlockwise
            elif -1.0<=u<-0.0 and 0.0<v<=1.0:
                ang=np.arctan(v/u)+cmath.pi
                ang_vel=((ang*linear_vel/(cmath.pi/2))-linear_vel)
                linear_vel=(linear_vel-((ang-cmath.pi/2)*linear_vel/(cmath.pi/2)))
            
            #Stick in the negative x axis, turns anticlockwise
            elif -1.0<=u<-0.0 and abs(v)==0.0:
                ang=cmath.pi
                ang_vel=linear_vel
                linear_vel=0
            
            #Stick in the third quadrant, moves backward and turns anticlockwise
            elif -1.0<=u<-0.0 and -1.0<=v<-0.0:
                ang=np.arctan(v/u)-cmath.pi
                ang_vel=-(linear_vel-(abs(ang)*linear_vel/(cmath.pi/2)))
                linear_vel=-(linear_vel-(abs(ang+cmath.pi/2)*linear_vel/(cmath.pi/2)))
            
            #Stick in the negative y axis, moves backforward
            elif -1.0<=v<=0.0 and abs(u)==0.0:
                ang=cmath.pi*(-1/2)
                ang_vel=0
                linear_vel=-linear_vel
            
            #Stick in the fourth quadrant, moves backward and turns clockwise
            elif 0.0<=u<=1.0 and -1.0<=v<0.0:
                ang=np.arctan(v/u)
                linear_vel=-linear_vel
                ang_vel=(abs(ang+cmath.pi/2)*linear_vel/(cmath.pi/2))
                linear_vel=(linear_vel-(abs(ang+cmath.pi/2)*linear_vel/(cmath.pi/2)))
            
            #Data visualization
            #self.get_logger().info("lin: " + str(linear_vel) + " ang: " + str(ang_vel)+ " ang: " + str(ang))
            #self.get_logger().info("but a: "+str(button_A)+"ba: "+str(ba)+" but b: "+str(button_B))
            #self.get_logger().info("r: "+str(rad)+" ang: "+str(ang)+" lin vel: " +str(linear_vel)+" ang vel: " +str(ang_vel))
            self.get_logger().info(str(foo.kinematicsCPU(linear_vel,ang_vel)))
           


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

