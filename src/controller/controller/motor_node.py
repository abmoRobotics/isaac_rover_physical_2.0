#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from exomy_msgs.msg import MotorCommands
import sys
sys.path.append('/home/xavier/isaac_rover_physical/exomy/scripts/utils')
from initRobot import Rover

class MotorNode(Node):
    """Convert Motor Commands"""

    def __init__(self):
        """Init Node."""
        self.node_name = 'motor_node'
        super().__init__(self.node_name)
        config_file = '/home/xavier/isaac_rover_physical/exomy/config/exomy.yaml'
        self.robot = Rover(config_file)

        # Create Subscription
        self.MotorSub = self.create_subscription(
            MotorCommands,
            'MotorCommands',
            self.callback,
            1)

        self.MotorSub  # prevent unused variable warning


        # Create watchdog timer
        self.watchdog_timer = self.create_timer(5.0, self.watchdog)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def __del__(self):
        self.robot.exit_handler()

    def callback(self, cmds):
        
        self.robot.setMotorsFromKinematics(cmds.steering_angles, cmds.motor_velocities)# Send motor values to motor controller

        self.watchdog_timer.cancel()
        # If this timer runs longer than the duration specified,
        # then watchdog() is called stopping the driving motors.
        # Preventing the robot to go on driving if connection is lost.
        self.watchdog_timer = self.create_timer(5.0, self.watchdog)

    def watchdog(self):
        self.get_logger().info('Watchdog fired. Stopping driving motors.')
        self.robot.exit_handler()


def main(args=None):
    rclpy.init(args=args)

    try:
        motor_node = MotorNode()
        try:
            rclpy.spin(motor_node)
        finally:
            motor_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
