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
        pins = [37, 33]
        GPIO.setwarnings(False)
        GPIO.setup(pins, GPIO.OUT)

        ###########################
        # Setup motor controllers #
        ###########################
        self.node = {'FL':1, 'FR':2, 'CL':3, 'CR':4, 'RL':5, 'RR':6, 'FL_EN':7, 'FR_EN':8, 'RL_EN':9, 'RR_EN':10}
        eds_path = 'src/controller/config/C5-E-2-09.eds'
        self.mode_oper = self.target_vl = self.mode_disp = self.status = self.control = self. DeltaSpeed = self.DeltaTime = {}

        for ID in self.node:
            self.get_logger().info(str(ID) + ' :' + str(self.node[ID]) )
            self.node[ID] = network.add_node(self.node[ID], eds_path)
            self.get_logger().info(str(self.node[ID]) )
            self.mode_oper[ID]  =    self.node[ID].sdo['Modes of operation']
            self.target_vl[ID]  =    self.node[ID].sdo['vl target velocity']
            self.mode_disp[ID]  =    self.node[ID].sdo['Modes of operation display']
            self.status[ID]     =    self.node[ID].sdo['Statusword']
            self.control[ID]    =    self.node[ID].sdo['Controlword']
            self.DeltaSpeed[ID] =    self.node[ID].sdo['vl velocity acceleration']['DeltaSpeed'] 
            self.DeltaTime[ID]  =    self.node[ID].sdo['vl velocity acceleration']['DeltaTime']

        # initilize the mode to velocity(2)
        self.mode_oper['FL'].phys = 0x02
        self.get_logger().info(str(self.mode_disp['FL'].phys))

        # Setting the acc
        self.DeltaSpeed['FL'].phys = 2000
        self.DeltaTime['FL'].phys = 10

        # Initilize the motors
        if self.mode_disp['FL'].phys == 2:
                self.target_vl['FL'].phys = 0
                self.control['FL'].phys = 0x0006
                self.get_logger().info('I am here: 1')
                if self.status['FL'].bits[0] == 1 and self.status['FL'].bits[5] == 1 and self.status['FL'].bits[9] == 1: 
                    self.control['FL'].phys = 0x0007
                    self.get_logger().info('I am here: 2')
                    if self.status['FL'].bits[0] == 1 and self.status['FL'].bits[1] == 1 and self.status['FL'].bits[4] == 1 and self.status['FL'].bits[5] == 1 and self.status['FL'].bits[9] == 1:
                        self.control['FL'].phys = 0x000F
                    else:
                        self.get_logger().info('I did not do it')


        self.subscription = self.create_subscription(
            MotorCommands,
            '/joy_listener/joystic_publisher',
            self.listener_callback,
            1000)
        

    def listener_callback(self, msg):
        
        self.get_logger().info('Publishing: "%s"' % str(msg.motor_linear_vel) + ' Ang: ' + str(msg.motor_angular_vel)+ ' Mode: ' +str(msg.mode))
        
        
        self.target_vl['FL'].phys = msg.motor_linear_vel * 2000

        

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