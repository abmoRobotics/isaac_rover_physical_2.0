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
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.VGA

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

    def callback(self):
        
        

        # Create and set RuntimeParameters after opening the camera
        runtime_parameters = sl.RuntimeParameters()
        runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # Use STANDARD sensing mode
        # Setting the depth confidence parameters
        runtime_parameters.confidence_threshold = 100
        runtime_parameters.textureness_confidence_threshold = 100

        # Capture 1 image and depth
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()

        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        tr_np = mirror_ref.m


        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            image_ocv = self.image.get_data()
            #cv.imwrite('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/image.jpg', image_ocv) 
            
            # Retrieve depth map. Depth is aligned on the left image
            self.zed.retrieve_image(self.depth, sl.VIEW.DEPTH)
            depth_ocv = self.depth.get_data()
            #cv.imwrite('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/depth.jpg', depth_ocv) 
            
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            # Save a file with all the points
            self.point_cloud.write("/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/pointcloud")
            
            sys.stdout.flush()

        #open pointcloud data
        pcloud = pypcd.PointCloud.from_path('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/pointcloud.pcd')
        np_pcloud = pcloud.pc_data.view(np.float32).reshape(pcloud.pc_data.shape + (-1,)) #convert to np array
        tf_cloud = TransCloud(np_pcloud) #apply transformation
        tf_cloud=np.float32(tf_cloud)

        transcloud = pypcd.make_xyz_rgb_point_cloud(tf_cloud) #convert to pointcloud
        #transcloud.save("transcloud2.pcd") #save the transformed pointcloud
        #transcloud.save('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/transcloud.pcd')

        heighdata=torch.tensor(tf_cloud,device="cuda:0")
        sampled_heightData=key_points(heighdata)

        sampled_heightData2=sampled_heightData.cpu().numpy()
        #np.save("heighdata",sampled_heightData2)

        #self.zed.close() # Close the camera
        return sampled_heightData2[:,2]

