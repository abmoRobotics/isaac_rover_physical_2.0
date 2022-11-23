#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from exomy_msgs.msg import CameraData
from sensor_msgs.msg import PointCloud2, PointCloud
import numpy as np
import time
import sys
import message_filters
sys.path.append('/home/xavier/isaac_rover_physical/exomy/scripts/utils')
sys.path.append('/home/xavier/ros2_numpy')
import ros2_numpy
from CameraSys import Cameras
import torch


class Camera_node(Node):
    """Convert Motor Commands"""

    def __init__(self):
        

        
        
        

        """Init Node."""
        self.node_name = 'Camera_node'
        super().__init__(self.node_name)
        self.pub = self.create_publisher( #Heightmap publisher
                CameraData,
                'CameraData',
                1)
        self.pointpub = self.create_publisher( #Publisher for visualizing pointcloud in Rviz
                PointCloud,
                'pointcloud',
                1)
        self.Keypointpub = self.create_publisher( #Publisher for visualizing keypoints in Rviz
                PointCloud,
                'Keypoint',
                1)

        self.cam1Sub = message_filters.Subscriber(self, PointCloud2, "/cam_1/depth/color/points")
        self.cam2Sub = message_filters.Subscriber(self, PointCloud2, "/cam_2/depth/color/points")

        queue_size = 1
        self.ts = message_filters.ApproximateTimeSynchronizer([self.cam1Sub, self.cam2Sub], queue_size, 0.8)
        self.ts.registerCallback(self.callback)


        """Init Camera."""
        self.camera = Cameras()
        
        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))
      
        
       

    def callback(self, data_cam1, data_cam2):
        try:
           
            start = time.perf_counter()  #returns the float value of time in s
            self.get_logger().info('\tRecived camera data new')
            start_numpify = time.perf_counter()
            pc1 = ros2_numpy.numpify(data_cam1) #converts ros messages to arrays
            pc2 = ros2_numpy.numpify(data_cam2)
            end_numpify = time.perf_counter() - start_numpify
            points=np.zeros((pc1.shape[0],3)) #empty matrix of all points extracted?
            points[:,0]=pc1['x']
            points[:,1]=pc1['y']
            points[:,2]=pc1['z']
            pcnp = np.array(points, dtype=np.float32)
            
            points2=np.zeros((pc2.shape[0],3))
            points2[:,0]=pc2['x']
            points2[:,1]=pc2['y']
            points2[:,2]=pc2['z']
            pcnp2 = np.array(points2, dtype=np.float32)


            pcnp = np.insert(pcnp, pcnp.shape[1], 1, axis=1)
            pcnp2 = np.insert(pcnp2, pcnp2.shape[1], 1, axis=1)
            start_transformation = time.perf_counter()
            points, Robotpos, RobotVel, RobotAcc, RobotRot, ang_vel, ang_acc, keypoints, elaps  = self.camera.callback(pcnp, pcnp2) ### 0.14s - 0.294s

            end_transformation = time.perf_counter() - start_transformation
            
            dataMsg = CameraData() 
            dataMsg.robot_pos = Robotpos.tolist()
            dataMsg.robot_vel = RobotVel.tolist()
            dataMsg.robot_acc = RobotAcc.tolist()
            dataMsg.robot_rot = RobotRot.tolist()
            
            start_message = time.perf_counter()
            torch.cuda.synchronize()
            #detach keypoints from tensor to numpy
            keypoints = keypoints.cpu().numpy() #tensor=numpy but in pytorch
            end_message = time.perf_counter() - start_message
           
            #Print robot tracking data
            #self.get_logger().info('\tKeyPoints: {}'.format(keypoints))
            #self.get_logger().info('\tRobot Rotation: {}'.format(RobotRot))
            #self.get_logger().info('\tRobot Angular Velocity: {}'.format(ang_vel))
            #self.get_logger().info('\tRobot Angular Acceleration: {}'.format(ang_acc))
            



            #Comment in the next section to be able to publish all point cloud data to ROS, to visualize it in Rviz
            #  
            # PointCloudTrans = PointCloud()
            # for i in range(len(points)):
            #     point = Point32()
            #     point.x = float(points[i][0])
            #     point.y = float(points[i][1])
            #     point.z = float(points[i][2])
            #     PointCloudTrans.points.append(point)
            # PointCloudTrans.header = data_cam1.header
            # self.pointpub.publish(PointCloudTrans)

            

            
            #Comment in the next section to be able to publish all keypoint data to ROS, to visualize it in Rviz
            #
            #keypoints_np = keypoints.cpu().detach().numpy()
            # SampledPointCloudTrans = PointCloud()
            # for data_point in keypoints:
            #     point = Point32()
            #     point.x = float(data_point[0])
            #     point.y = float(data_point[1])
            #     point.z = float(data_point[2])
            #     SampledPointCloudTrans.points.append(point)

            # SampledPointCloudTrans.header = data_cam1.header
            # self.Keypointpub.publish(SampledPointCloudTrans)
            
            
            keypoints = np.delete(keypoints, 0, 1) 
            keypoints = np.delete(keypoints, 0, 1)
            
            keypoints = keypoints.flatten().tolist() #get the matrix in 1d

            
            dataMsg.depth_data = keypoints
            

            start_publish = time.perf_counter()
            self.pub.publish(dataMsg)
            end_publish = time.perf_counter() - start_publish
            
            end = time.perf_counter()- start
            # self.get_logger().info('\tTime transformation: {}'.format(end_transformation))
            # self.get_logger().info('\tTime numpify : {}'.format(end_numpify))
            # self.get_logger().info('\tTime message : {}'.format(end_message))
            # self.get_logger().info('\tTime publish : {}'.format(end_publish))
            # self.get_logger().info('\tTime to tensor : {}'.format(elaps))
            # self.get_logger().info('\tTime camera node - transformation : {}'.format(end))

        except Exception as e: 
           self.get_logger().info('\tERROR: {}'.format(e))

def main(args=None):
    rclpy.init(args=args)

    try:
        CameraNode = Camera_node()
        try:
            rclpy.spin(CameraNode)
        finally:
            CameraNode.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()