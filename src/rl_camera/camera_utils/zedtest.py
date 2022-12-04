#!/usr/bin/env python3
# zedtest.py
import pyzed.sl as sl
import cv2 as cv
import math
import numpy as np
import sys
import matplotlib.image
import torch
from pypcd import pypcd
from camera_utils import TransCloud, heightmap_distribution, limit_x, key_points



class ZedCamera:
    def __init__(self):
        #self.heightmap_distribution = torch.tensor(heightmap_distribution( 1.12, 1.2, square=True, y_start=0.03, delta=0.05, front_heavy=0.0), device='cuda:0')
        # Create a Camera object
        self.zed = sl.Camera()
        
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.VGA

        # Open the camera
        err = self.zed.open(init_params)
        print("ok")
        if err != sl.ERROR_CODE.SUCCESS:
            print("in")
            exit(1)
            print("exit")
        
        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        # Capture 1 image and depth
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.data_structure = []

        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        tr_np = mirror_ref.m

    def callback(self):
        self.data_structure=[]
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            image_ocv = self.image.get_data()
            cv.imwrite('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/image.jpg', image_ocv) 
            
            # Retrieve depth map. Depth is aligned on the left image
            # self.zed.retrieve_image(self.depth, sl.VIEW.DEPTH)
            # depth_ocv = self.depth.get_data()
            #cv.imwrite('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/depth.jpg', depth_ocv) 
            
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            # Save a file with all the points
            self.point_cloud.write("/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/pointcloud2")
            point_cloud_data = self.point_cloud.get_data()
            
        for i in range(len(point_cloud_data)):
            self.data_structure = np.append(self.data_structure,point_cloud_data[i,:])
        x=self.data_structure[0::4]
        y=self.data_structure[1::4]
        z=self.data_structure[2::4]
        color=self.data_structure[3::4]
        pcloud_matrix=np.array([x,y,z,color])
        pcloud_matrix=np.transpose(pcloud_matrix)
        # print(pcloud_matrix)
        # print(pcloud_matrix.shape)
            #point_cloud_data = point_cloud_data.pc_data.view(np.float32).reshape(point_cloud_data.pc_data.shape + (-1,))
            #point_cloud_data2 = pypcd.make_xyz_rgb_point_cloud(point_cloud_data)
            #np.save('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/pointcloud4.pcd',point_cloud_data)
            #sys.stdout.flush()


        #open pointcloud data
        # pcloud = pypcd.PointCloud.from_path('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/pointcloud.pcd')
        # np_pcloud = pcloud.pc_data.view(np.float32).reshape(pcloud.pc_data.shape + (-1,)) #convert to np array
        tf_cloud = TransCloud(pcloud_matrix) #apply transformation
        tf_cloud=np.float32(tf_cloud)
        # print(tf_cloud)
        # print(tf_cloud.shape)

        tf_cloud = np.delete(tf_cloud, 3, 1)
        transcloud = pypcd.make_xyz_point_cloud(tf_cloud) #convert to pointcloud
        #transcloud.save("transcloud.pcd") #save the transformed pointcloud
        transcloud.save('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/transcloud.pcd')
        # np.save('try',tf_cloud)
  
        #tf_cloud = np.delete(tf_cloud, 3, 1)
   
        # print(tf_cloud)
        print("here")
        heightdata=torch.tensor(tf_cloud,device="cuda:0")
        # print(heightdata.shape)
        sampled_heightData=key_points(heightdata)
        # print(sampled_heightData.shape)
        sampled_heightData2=sampled_heightData.cpu().numpy()

        np.save("/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/heighdata",sampled_heightData2)
        #print(sampled_heightData2)
        #self.zed.close() # Close the camera
        
        return sampled_heightData2

def main(args=None):
    camera=ZedCamera()
    camera.callback()
 
    



if __name__ == '__main__':
    main()