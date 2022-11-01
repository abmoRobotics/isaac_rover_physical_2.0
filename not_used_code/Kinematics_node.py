import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rover_msgs.msg import MotorCommands, Actions
import sys
sys.path.append('/home/xavier/isaac_rover_physical/exomy/scripts/utils')
from kinematicsCPU import kinematicsCPU #Kinematics

class Kinematics_node(Node):
    """Convert Motor Commands"""

    def __init__(self):
        """Init Node."""
        self.node_name = 'Kinematics_node'
        super().__init__(self.node_name)

        # Create Subscription
        self.ActionSub = self.create_subscription(
            Actions,
            'Actions',
            self.callback,
            1)

        self.ActionSub  # prevent unused variable warning

        self.MotorPub = self.create_publisher(
            MotorCommands,
            'MotorCommands',
            1)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))


    def callback(self, cmds):
        Message = MotorCommands()
        Message.steering_angles, Message.motor_velocities = kinematicsCPU(cmds.lin_vel, cmds.ang_vel) #Use Ackermann kinematics to find motor values

        self.MotorPub.publish(Message)



def main(args=None):
    rclpy.init(args=args)

    try:
        kinematicsNode = Kinematics_node()
        try:
            rclpy.spin(kinematicsNode)
        finally:
            kinematicsNode.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
