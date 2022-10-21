#!/usr/bin/env python3

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
        pins = [37, 33]
        GPIO.setwarnings(False)
        GPIO.setup(pins, GPIO.OUT)

        ###########################
        # Setup motor controllers #
        ###########################

        node = network.add_node(1, 'src/controller/config/C5-E-2-09.eds')
        self.mode_oper= node.sdo['Modes of operation']
        self.target_vl = node.sdo['vl target velocity']
        self.mode_disp = node.sdo['Modes of operation display']
        self.status = node.sdo['Statusword']
        self.control = node.sdo['Controlword']
        self.DeltaSpeed = node.sdo['vl velocity acceleration']['DeltaSpeed'] 
        self.DeltaTime = node.sdo['vl velocity acceleration']['DeltaTime']

        # initilize the mode to velocity(2)
        self.mode_oper.phys = 2
        
        # Setting the acc
        self.DeltaSpeed.phys = 2000
        self.DeltaTime.phys = 10

        # Initilize the motors
        if self.mode_disp.phys == 0x0002:
                self.target_vl.phys = 0
                self.control.phys = 0x0006
                if self.status.bits[0] == 1 and self.status.bits[5] == 1 and self.status.bits[9] == 1: 
                    self.control.phys = 0x0007
                    if self.status.bits[0] == 1 and self.status.bits[1] == 1 and self.status.bits[4] == 1 and self.status.bits[5] == 1 and self.status.bits[9] == 1:
                        self.control.phys = 0x000F
                    else:
                        self.get_logger().info('I did not do it')


        self.subscription = self.create_subscription(
            MotorCommands,
            '/joy_listener/joystic_publisher',
            self.listener_callback,
            1000)
        

    def listener_callback(self, msg):
        
        self.get_logger().info('Publishing: "%s"' % str(msg.motor_linear_vel) + ' Ang: ' + str(msg.motor_angular_vel)+ ' Mode: ' +str(msg.mode))
        
        
        self.target_vl.phys = msg.motor_linear_vel * 2000

        

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