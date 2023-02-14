#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import MotorCommands
import numpy as np
import cmath
import sys
import torch
from skrl.agents.torch.ppo import PPO

# This is very ugly! - Fix at some point
sys.path.insert(0,'/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/camera_utils')
sys.path.insert(0,'/home/orin/Documents/isaac_rover_physical_2.0/src/controller/controller/utils')
from loadpolicy import student_loader, teacher_loader
from zedtest_realsense import ZedCamera
from ros_server import ServerNode

#from utils.model import StochasticActorHeightmap, DeterministicHeightmap

#It makes possible to see the angles and velocities in each robot wheel
#spec=importlib.util.spec_from_file_location("kinematicsCPU","/home/orin/Documents/isaac_rover_physical_2.0/src/controller/scripts/kinematicsCPU.py")
#foo=importlib.util.module_from_spec(spec)
#spec.loader.exec_module(foo)

#setup variables

max_speed = 1
global mode, autom, rad,new_max,new_min,linear_vel,ang,ang_vel, turn_around
mode        =   4 # Default it is set to 0.25 percentage speed mode.
autom       =   0
rad         =   0
turn_around =   0
new_max     =   0
new_min     =   0
linear_vel  =   0
ang         =   0
ang_vel     =   0
rot_speed   =   0
#Joystick is the publisher (sends velocities)Motor node is the subscriber.

#When the joystick has 0 lin_vel and 0 ang_vel the robot will turn the wheels to the initial position

class joy_listener(Node):

    def __init__(self):
        super().__init__('joy_listener')
        self.get_logger().info('Joy_listener initializing')
        # ENABLE INTERFACE HERE
        self.is_remote_interface_enabled = False
        self.subscription = self.create_subscription(Joy, '/joy/joy', self.listener_callback,1000)
    
        self.publisher_ = self.create_publisher(MotorCommands, '/joy_listener/joystic_publisher', 1000)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.prev_actions = torch.tensor([[0,0]], device = 'cuda:0')
        self.mode = 0
        self.power_off = 0 
        self.turn_around = 0
        self.get_logger().info('Joy_listener initialized')
        self.camera = ZedCamera(debug = False, device = "cpu", nulls = 0.0, goal = [4.0, 0.0])
        if self.is_remote_interface_enabled:
            self.server = ServerNode()
        self.get_logger().info('Heightmap: %s, ' % str(self.camera.heightmap.shape))
        info = {
            "reset": 0,
            "actions": 2,
            "proprioceptive": 4,
            "sparse": self.camera.map.get_num_sparse_vector(),
            "dense": self.camera.map.get_num_dense_vector()}
        

        self.student = student_loader(info, "model1")
        #self.student2 = student_loader(info, "model2")
        self.teacher = teacher_loader(info, "model1")

        self.log_data = []
        self.log_file = open('logfile.txt', 'w')

    def timer_callback(self):

        msg = MotorCommands()

        # goal_dist, goal_ang, heightdata = self.camera.callback()
        # self.get_logger().info('Goal_dist: %s, ' % str(goal_dist) + 'Goal_angle: %s, ' % str(goal_ang))

        if self.mode == 0: # Manual driving
            
            xt, yt, dist_total, goal_dist, goal_ang, heightdata, image = self.camera.callback()
            self.get_logger().info('Goal_dist: %s, ' % str(goal_dist) + 'Goal_angle: %s, ' % str(goal_ang) + 'Dist traveled: %s, ' % str(dist_total))
            # TODO: sent image to server to send it
            if self.is_remote_interface_enabled:
                self.server.send_image(image)
            # Set message to motors
            msg.motor_linear_vel = self.lin_vel * 2
            msg.motor_angular_vel = self.ang_vel * 2
            msg.mode = self.mode
            msg.power_off = self.power_off
            msg.turn_around = self.turn_around
        
        elif self.mode == 1 or self.mode == 2: # Autonomous driving

            # Get heightmap from camera
            tx, ty, dist_total, goal_dist, goal_ang, heightdata, image = self.camera.callback()
            self.get_logger().info('Goal_dist: %s, ' % str(goal_dist) + 'Goal_angle: %s, ' % str(goal_ang) + 'Dist traveled: %s, ' % str(dist_total))
            # TODO: sent image to server to send it
            if self.is_remote_interface_enabled:
                self.server.send_image(image)


            #self.get_logger().info('Heigtdata shape: %s, ' % str(heightdata[:,2].unsqueeze(0).shape))
            sparse = self.camera.map.get_sparse_vector(heightdata[:,2].unsqueeze(0))
            dense = self.camera.map.get_dense_vector(heightdata[:,2].unsqueeze(0))
            #sparse[:,:] = 0.00
            #dense[:,:] = 0.00
            # Convert to a single tensor

            #sparse = torch.where(sparse < -0.05 , torch.zeros_like(sparse), sparse)
            #dense = torch.where(dense < -0.05 , torch.zeros_like(dense), dense)

            # self.get_logger().info('%s, ' % str(sparse))

            # self.prev_actions[:,0] = 0.9
            # self.prev_actions[:,1] = 0.3
            goal_data = torch.tensor((goal_dist/9.0, goal_ang/3.1415), device = 'cuda:0').unsqueeze(0)
            #goal_data[:,0] = 0.4
            #goal_data[:,1] = -1.0
            obs = torch.cat((goal_data, self.prev_actions, sparse, dense), dim = 1).float()
            
            #obs[:,:] = 0
            # # Make inferance of network
            if self.mode == 1: # Trainer network - X
                actions = self.teacher.act(obs).squeeze()
                #self.get_logger().info('Lin_vel: %s, ' % str(actions.shape) + 'Ang_vel: %s, ' % str(self.prev_actions.shape))
                #actions = torch.where(torch.abs(actions-self.prev_actions[0])<  torch.ones(2,device='cuda:0')*0.1,self.prev_actions[0], actions)
            if self.mode == 2: # Student network - A
                actions = self.student.act(obs).squeeze()
            if self.mode == 3: # Student2 network - B
                actions = self.student2.act(obs).squeeze()
            
            
            #self.get_logger().info('Input: %s, ' % str(obs[0:4]))            
            #self.get_logger().info('Lin_vel: %s, ' % str(actions[0].float()) + 'Ang_vel: %s, ' % str(actions[1].float()))
            
            #self.get_logger().info('RUNNINGG')
            self.log_file.write("%s\n" % str([tx, ty, actions[0].item(), actions[1].item(), dist_total]))

            self.prev_actions = actions.unsqueeze(0)
           # actions = actions * 9
            # Set message to motors
            msg.motor_linear_vel = actions[0].item()
            msg.motor_angular_vel = actions[1].item()
            msg.mode = self.mode
            msg.power_off = self.power_off
            msg.turn_around = self.turn_around
                        # If distance is below threshold, shift to manual mode.
            if goal_dist < 0.4:
                self.mode = 0
                self.autom = 0
                self.get_logger().info('Finished')
        else:
            self.get_logger().info('Invalid operating mode!')
        
        # Publish message to motors
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % str(msg.motor_linear_vel) + str(msg.motor_angular_vel) )

    def listener_callback(self, msg):
        global autom,mode,ang,linear_vel,ang_vel,rad,new_max,new_min,rot_speed

        #IMPORTANT: mode button changes the joystick's mapping. Make sure the little led next to it is off 

        LR_stick = msg.axes[0] # Moving the robot left and right  
        UD_stick = msg.axes[1] # Moving the robot forward and backward  

        #Maximum working speed related to the maximum motor speed
        LR_cross = msg.axes[6]  # Left = 0 to +/-50 % right = 0 to +/-75%  
        UP_cross = msg.axes[7]  # Down = 0 to +/-25% UP = 0 to +/-100%   

        button_A = msg.buttons[0] # Starts the automatic(Student) mode
        button_B = msg.buttons[1] # Unused
        button_X = msg.buttons[2] # Starts the automatic(Trainer) mode
        button_Y = msg.buttons[3] # Manual mdode

        button_LB = msg.buttons[4] # It powers off the motors and node
        button_RB = msg.buttons[5]
        
        butten_LT = msg.axes[2] - 1 # Increse or decrese the speed of the robot to move either forward or backward
                                    #In turning mode, makes the robot rotate clockwise
        butten_RT = msg.axes[5]-1 # Only available in turning mode, rotating anticlockwise


        #Control the manual and automatic mode
        if button_Y==1:
            autom=0
            self.mode = int(autom)
        elif button_X==1:
            autom=1
            self.mode = int(autom)
        elif button_A==1:
            autom = 2
            self.mode = int(autom)
        elif button_B==1:
            autom = 3
            self.mode = int(autom)
        
        #Control the maximum speed mode

        if button_LB ==1:
            self.power_off = int(1)
            #self.get_logger().info("power: " + str(self.power_off)) 
        else:
            self.power_off = int(0)


        if LR_cross == -1.0 :
            mode=1

        elif LR_cross == 1.0 :
            mode=2

        elif UP_cross == -1.0 :
            mode=4
        
        elif UP_cross == 1.0 :
            mode=3

        #Control the driving mode
        #elif button_A==1:
        #    self.turn_around=1
        #elif button_B==1:
        #    self.turn_around=0    


        #Coordinates in the joystick
        u=-LR_stick
        v=UD_stick
        
        #Transformation to square plane coordinates (not in use)
        #x=1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)+2*u*math.sqrt(2)))-1/2*math.sqrt(abs(2+math.pow(u,2)-math.pow(v,2)-2*u*math.sqrt(2)))

        #y=1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)+2*v*math.sqrt(2)))-1/2*math.sqrt(abs(2-math.pow(u,2)+math.pow(v,2)-2*v*math.sqrt(2)))
        

        #Drive the robot with the joystick
        #All the control goes accordingly the kinematicsCPU code
        if autom==0:
            
            if mode == 2:
                new_max = ((0.75 * max_speed)*butten_RT)/-2
                new_min = ((0.75 * max_speed)*butten_LT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min

            elif mode == 1:
                new_max = ((0.50 * max_speed)*butten_RT)/-2
                new_min = ((0.50 * max_speed)*butten_LT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min

            elif mode == 3:
                new_max = ((1.00 * max_speed)*butten_RT)/-2
                new_min = ((1.00 * max_speed)*butten_LT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min
            
            elif mode == 4:
                new_max = ((0.25 * max_speed)*butten_RT)/-2
                new_min = ((0.25 * max_speed)*butten_LT)/-2
                linear_vel=new_max
                rot_speed=new_max-new_min
               
            
            #When the stick is in equilibrium position
            if 0.0==abs(u) and abs(v)==0.0:

                #pressing LT rotates clockwise, pressing RT rotates anticlockwise
                if self.turn_around==1:
                    ang_vel=-rot_speed
                    linear_vel=0
                else:
                    ang=0
                    ang_vel=0
                    linear_vel=0

            #Blocks free driving when it is in turning mode
            elif self.turn_around==1 and 0.0!=abs(u) and abs(v)!=0.0:
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
            #self.get_logger().info(str(foo.kinematicsCPU(linear_vel,ang_vel)))
            self.lin_vel = float(linear_vel)
            self.ang_vel = float(ang_vel)
        


def main(args=None):
    rclpy.init(args=args)

    joy_lis = joy_listener()

    joy_lis
    rclpy.spin(joy_lis)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_lis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()