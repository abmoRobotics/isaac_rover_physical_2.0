#!/usr/bin/env python3
# zedtest.py
import pyzed.sl as sl
import cv2 as cv
import math
import numpy as np
import sys
import matplotlib.image
import time
import torch
from pypcd import pypcd
from CameraFunctions import TransCloud, key_points, TransCloudTensor
from heightmap_distribution import Heightmap


class ZedCamera:
    def __init__(self, debug = False, device = "cuda:0", nulls = -3.0):

        self.debug = debug
        self.device = device

        self.map=Heightmap("cuda:0")
        self.heightmap = self.map.point_distribution
        self.nulls = nulls

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.VGA

        # Create a Camera object and open
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("There was an error opening the ZED camera. Is it connected?")
            exit(1)
        
        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 98
        self.runtime_parameters.textureness_confidence_threshold = 98
        self.runtime_parameters.remove_saturated_areas = True

        # Initialize image and depth
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.data_structure = []


    def callback(self):
        start2 = time.perf_counter()

        self.data_structure=[]
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            image_ocv = self.image.get_data()

            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        
            # Cast data of sl.Mat -> Numpy array -> Tensor
            point_cloud_data = self.point_cloud.get_data()
            pcloud_matrix = np.reshape(point_cloud_data[:,:,0:3], (-1,3)) # Reshape to a (-1,3) shaped numpy
            pcloud_matrix= pcloud_matrix[::4]
            pcloud_matrix_tensor = torch.tensor(pcloud_matrix,device=self.device,dtype=torch.float64)

            # Transform
            tf_cloud_tensor = TransCloudTensor(pcloud_matrix_tensor, device=self.device)

            # Extract heightmap
            sampled_heightData=key_points(tf_cloud_tensor, self.heightmap, device="cuda:0", nulls = self.nulls)
            
            if self.debug:
                with_color = torch.tensor(np.reshape(point_cloud_data[:,:,0:4], (-1,4))[::4],device=self.device)
                torch.save(with_color, 'Original.pt')
                torch.save(tf_cloud_tensor, 'Transformed.pt')
                torch.save(sampled_heightData, 'Heightmap.pt')
            
            torch.cuda.synchronize()

            return 1
            
        else:
            return 0


if __name__ == '__main__':
    TestTensor = torch.tensor([2,3,1,31],device="cpu")
    camera=ZedCamera(debug = True, device = "cpu", nulls = 0.0)
    
    print(camera.map.point_distribution.shape[0], " points in heightmap distribution")
    print("Initialized class")
    camera.callback()
    # for i in range(10):
    #     start = time.perf_counter()
    #     camera.callback()
    #     print("Done: ", time.perf_counter()-start)
