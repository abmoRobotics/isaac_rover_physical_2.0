#!/usr/bin/env python3
import os
import signal
import Jetson.GPIO as GPIO 
import rclpy
import canopen 
from canopen import *

from rclpy.node import Node

from rover_msgs.msg import MotorCommands
import controller_utils


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
        global FL, FR, CL, CR, RL, RR, FL_ang, FR_ang, RL_ang, RR_ang
        FL, FR, CL, CR, RL, RR, FL_ang, FR_ang, RL_ang, RR_ang = range(0,10) # Define the ID's
        ID = [FL, FR, CL, CR, RL, RR, FL_ang, FR_ang, RL_ang, RR_ang]
        ID_hall = [FL, FR, CL, CR, RL, RR]
        ID_en = [FL_ang, FR_ang, RL_ang, RR_ang]

        self.get_logger().info(str(ID))
        self.mode_oper  =   []
        self.target_vl  =   []
        self.mode_disp  =   []
        self.status     =   []
        self.control    =   []
        self.actual_vl  =   [] 
        self.target_pos =   []

        controller_utils.autosetup(ID_hall)   # Auto-setup for the motors so they all have the same
        controller_utils.autosetup_encoder(ID_en)

        for id in ID:
            
            self.node = network.add_node(id+1, eds_path)
            self.get_logger().info('NodeID: ' + str(id + 1) )

            self.mode_oper.append(self.node.sdo['Modes of operation'])
            self.target_vl.append(self.node.sdo['vl target velocity'])
            self.mode_disp.append(self.node.sdo['Modes of operation display'])
            self.status.append(self.node.sdo['Statusword'])
            self.control.append(self.node.sdo['Controlword'])
            self.actual_vl.append(self.node.sdo[0x6044])


            self.node.sdo[0x2030].phys  = 6             # pole pair = 6      step angle = 15
            self.node.sdo[0x2031].phys  = 14000         # maximum permissible motor current in mA
            self.node.sdo[0x6075].phys  = 5000          # rated current of the motor in mA 
            self.node.sdo[0x6073].phys  = 15000         # maximum current in mA
            self.node.sdo[0x3202].raw   = 0x00000041    # Motor type = BLDC with closed loop 
            

            self.node.sdo[0x60A8].raw   = 0x00100000    # Position unit as radian
            self.node.sdo[0x60A9].raw   = 0x00B44700    # Position unit as revolution / per minuts
            self.node.sdo[0x604C][0x01].phys  = 60      # Vl Dimension Factor Numerator
            self.node.sdo[0x604C][0x02].phys  = 1       # Vl Dimension Factor Denominato







            if id >= 7 :
                # initilize the mode to Profile Position
                #self.mode_oper[id].phys = 6

                # if self.mode_disp[id].phys == 6:
                #     if self.status[id].bits[13] == 0 and self.status[id].bits[12] == 0 and self.status[id].bits[10] == 0:
                #         self.get_logger().info('Homeing is performed on NodeID' + str(id + 1))

                        
                #         if self.status[id].bits[13] == 0 and self.status[id].bits[12] == 1 and self.status[id].bits[10] == 1:
                #             self.get_logger().info('Homeing completed on NodeID' + str(id + 1))

                self.mode_oper[id].phys = 0x01
                self.get_logger().info('Position ID: ' + str(id + 1) )

<<<<<<< HEAD
                self.mode_oper[id].phys = 0x01
=======
                self.target_pos[id] = self.node.sdo[0x607A]
>>>>>>> a62cc1f3064ba10f5aa58bbc7d95760e3609f3e5

                #self.node.sdo[0x6083].phys # Desired starting acceleration
                #self.node.sdo[0x6083].phys # Desired braking deceleration
        
                # Initilize the position motors
                if self.mode_disp[id].phys == 1:
                    self.target_pos[id].phys = 0
                    self.control[id].bits[4] = 0
                    if self.status[id].bits[12] == 1:
                        self.get_logger().info('The  target angle is valid')
                        while self.status[id].bits[10] == 0:
                            self.get_logger().info('The motor is moving')

                        
                                


            elif id < 7:        
                # initilize the mode to velocity

                self.mode_oper[id].phys = 0x02
                self.get_logger().info('Velocity ID: ' + str(id + 1) )

                # Setting the acceleration
                self.node.sdo['vl velocity acceleration']['DeltaSpeed'].phys = 3500
                self.node.sdo['vl velocity acceleration']['DeltaTime'].phys = 250

                # Setting the deceleration
                self.node.sdo[0x6049][0x01].phys = 3500
                self.node.sdo[0x6049][0x02].phys = 100

                # Initilize the velocity motors
                if self.mode_disp[id].phys == 2:
                        self.target_vl[id].phys = 0
                        self.control[id].phys = 0x0006
                        if self.status[id].bits[0] == 1 and self.status[id].bits[5] == 1 and self.status[id].bits[9] == 1: 
                            self.control[id].phys = 0x0007
                            if self.status[id].bits[0] == 1 and self.status[id].bits[1] == 1 and self.status[id].bits[4] == 1 and self.status[id].bits[5] == 1 and self.status[id].bits[9] == 1:
                                self.control[id].phys = 0x000F
                            else:
                                self.get_logger().info('I did not do it')


        self.subscription = self.create_subscription(
            MotorCommands,
            '/joy_listener/joystic_publisher',
            self.listener_callback,
            10)
        

    def listener_callback(self, msg):
        
        
        self.get_logger().info(' Node3: "%s"' % str(self.actual_vl[CL].phys) + 'Node1 "%s"' % str(self.actual_vl[FL].phys) + ' Node2: "%s"' % str(self.actual_vl[FR].phys))
        
        steering_angles, motor_velocities = controller_utils.Ackermann(msg.motor_linear_vel, msg.motor_angular_vel, device='cpu')
        #self.get_logger().info(' Vel controller: "%s"' % str(msg.motor_linear_vel) + 'Velocity "%s"' % str(motor_velocities))
        #self.target_pos[FL_ang].phys = steering_angles[FL]
        #self.target_pos[FR_ang].phys = steering_angles[FR]
        #self.target_pos[RL_ang].phys = steering_angles[RL]
        #self.target_pos[RR_ang].phys = steering_angles[RR]

        self.target_vl[FL].phys = (motor_velocities[FL] * 9.549297) 
        self.target_vl[FR].phys = (-motor_velocities[FR] * 9.549297)
        self.target_vl[CL].phys = (motor_velocities[CL] * 9.549297)
        self.target_vl[CR].phys = (-motor_velocities[CR] * 9.549297) 
        self.target_vl[RL].phys = (motor_velocities[RL] * 9.549297) 
        self.target_vl[RR].phys = (-motor_velocities[RR] * 9.549297) 
        
        if msg.power_off == 1:
            self.control[FL].phys = 0
            self.control[FR].phys = 0  
            self.control[CL].phys = 0 
            self.control[CR].phys = 0
            self.control[RL].phys = 0 
            self.control[RR].phys = 0  
            self.node.sdo[0x1011][0x05].phys = 0x64616F6C # Restart the drive 
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getppid(), signal.CTRL_C_EVENT)
        
        elif msg.turn_around == 1:
<<<<<<< HEAD
            self.target_pos[FL_ang].phys = steering_angles[FL]
            self.target_pos[FR_ang].phys = steering_angles[FR]
            self.target_pos[RL_ang].phys = steering_angles[RL]
            self.target_pos[RR_ang].phys = steering_angles[RR]
=======
            #self.target_pos[FL_ang].phys = steering_angles[FL]
            #self.target_pos[FR_ang].phys = steering_angles[FR]
            #self.target_pos[RL_ang].phys = steering_angles[RL]
            #self.target_pos[RR_ang].phys = steering_angles[RR]
            pass
>>>>>>> a62cc1f3064ba10f5aa58bbc7d95760e3609f3e5
        

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