#!/usr/bin/env python3

from re import I
import Jetson.GPIO as GPIO 
import canopen 
from canopen import *
import rclpy
from rclpy.node import Node
from rover_msgs.msg import MotorCommands


class motor_subscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        ############
        # Settings #
        ############
        self.max_linear_vel = 2000
        self.max_angular_vel = 400

        ####################
        # Open can network #
        ####################
        network = canopen.Network()
        network.connect(channel='can1', bustype='socketcan')
        GPIO.setmode(GPIO.BOARD)

        ###########################
        # Setup motor controllers #
        ###########################
        eds_path = 'src/controller/config/C5-E-2-09.eds'
        global FL, FR, CL, CR, RL, RR, FL_EN, FR_EN, RL_EN, RR_EN
        FL, FR, CL, CR, RL, RR, FL_EN, FR_EN, RL_EN, RR_EN = range(0,10) 

        ID = range(0,1)
        self.mode_oper  =   []
        self.target_vl  =   []
        self.mode_disp  =   []
        self.status     =   []
        self.control    =   []
        self.DeltaSpeed =   []
        self.DeltaTime  =   []


        for id in ID:
            self.node = network.add_node(id + 1, eds_path)
            self.get_logger().info('NodeID: ' +str(id + 1) )
            self.mode_oper.append(self.node.sdo['Modes of operation'])
            self.target_vl.append(self.node.sdo['vl target velocity'])
            self.mode_disp.append(self.node.sdo['Modes of operation display'])
            self.status.append(self.node.sdo['Statusword'])
            self.control.append(self.node.sdo['Controlword'])
            self.DeltaSpeed.append(self.node.sdo['vl velocity acceleration']['DeltaSpeed'])
            self.DeltaTime.append(self.node.sdo['vl velocity acceleration']['DeltaTime'])

            # initilize the mode to velocity(2)
            self.mode_oper[id].phys = 0x02

            # Setting the acc
            self.DeltaSpeed[id].phys = 2000
            self.DeltaTime[id].phys = 10

            # Initilize the motors
            if self.mode_disp[id].phys == 2:
                    self.target_vl[id].phys = 0
                    self.control[id].phys = 0x0006
                    self.get_logger().info('I am here: 1')
                    if self.status[id].bits[0] == 1 and self.status[id].bits[5] == 1 and self.status[id].bits[9] == 1: 
                        self.control[id].phys = 0x0007
                        self.get_logger().info('I am here: 2')
                        if self.status[id].bits[0] == 1 and self.status[id].bits[1] == 1 and self.status[id].bits[4] == 1 and self.status[id].bits[5] == 1 and self.status[id].bits[9] == 1:
                            self.control[id].phys = 0x000F
                        else:
                            self.get_logger().info('I did not do it')


        self.subscription = self.create_subscription(
            MotorCommands,
            '/joy_listener/joystic_publisher',
            self.listener_callback,
            1000)
        

    def listener_callback(self, msg):
        
        self.get_logger().info('Publishing: "%s"' % str(msg.motor_linear_vel) + ' Ang: ' + str(msg.motor_angular_vel)+ ' Mode: ' + str(msg.mode))
        
        
        self.target_vl[FL].phys = msg.motor_linear_vel * 2000

        if msg.power_off == 1:
            self.control[FL].phys = 0 
            self.destroy_node()
            rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    motor_sub = motor_subscriber()


    motor_sub.get_logger().info(f'{GPIO.JETSON_INFO}')
    rclpy.spin(motor_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    motor_sub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()