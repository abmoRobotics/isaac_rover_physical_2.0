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
    def __init__(self, debug = False, device = "cuda:0", nulls = -3.0, goal = [-3, 0]):

        self.debug = debug
        self.device = device

        self.map=Heightmap("cuda:0")
        self.heightmap = self.map.point_distribution
        self.nulls = nulls

        # Camera serial numbers
        zed_2_serial    = 37915676
        zed_mini_serial = 10029318

        # # Create a InitParameters object for ZED2 and set configuration parameters
        # init_params = sl.InitParameters()
        # init_params.set_from_serial_number(zed_2_serial)
        # init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        # #init_params.coordinate_system = sl.COORDINATE_SYSTEM_IMAGE
        # init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        # init_params.camera_resolution = sl.RESOLUTION.VGA

        # Create a InitParameters object for ZED_MINI and set configuration parameters
        init_params_mini = sl.InitParameters(coordinate_units=sl.UNIT.METER,)
        init_params_mini.set_from_serial_number(zed_mini_serial)
        #init_params_mini.coordinate_system = sl.COORDINATE_SYSTEM_IMAGE
        # init_params_mini.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params_mini.camera_resolution = sl.RESOLUTION.HD720
        init_params_mini.camera_fps = 0        

        # # Create a Camera object for ZED2 and open
        # self.zed = sl.Camera()
        # status = self.zed.open(init_params)
        # if status != sl.ERROR_CODE.SUCCESS:
        #     print("There was an error opening the ZED. Is it connected1?")
        #     exit(1)

        # Create a Camera object for ZED_MINI and open
        self.zed_mini = sl.Camera()
        status_mini = self.zed_mini.open(init_params_mini)
        if status_mini != sl.ERROR_CODE.SUCCESS:
            print("There was an error opening the ZED_MINI. Is it connected?")
            exit(1)
        
        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 98
        self.runtime_parameters.textureness_confidence_threshold = 98
        self.runtime_parameters.remove_saturated_areas = True

               # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters2 = sl.RuntimeParameters()
        self.runtime_parameters2.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters2.confidence_threshold = 98
        self.runtime_parameters2.textureness_confidence_threshold = 98
        self.runtime_parameters2.remove_saturated_areas = True

        # Initialize image and depth
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.data_structure = []

        # Initilize for camera position tracker
        self.goal = goal
        self.zed_pose = sl.Pose()
        self.zed_sensors = sl.SensorsData()
        initial_trans = sl.Transform()
        initial_trans.set_euler_angles(0,0,0,radian=False)
        
        # Enable positional tracking for ZED_MINI with default parameters
        py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        tracking_parameters.enable_imu_fusion = False
        #tracking_parameters.set_initial_world_transform(initial_trans)
        err = self.zed_mini.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)


    def callback(self):

        self.data_structure=[]

        #if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS and self.zed_mini.grab(self.runtime_parameters2) == sl.ERROR_CODE.SUCCESS:
        if self.zed_mini.grab(self.runtime_parameters2) == sl.ERROR_CODE.SUCCESS:
            
            # # Retrieve left image
            # self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            # image_ocv = self.image.get_data()

            # # Retrieve colored point cloud. Point cloud is aligned on the left image.
            # self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        
            # # Cast data of sl.Mat -> Numpy array -> Tensor
            # point_cloud_data = self.point_cloud.get_data()
            # pcloud_matrix = np.reshape(point_cloud_data[:,:,0:3], (-1,3)) # Reshape to a (-1,3) shaped numpy
            # pcloud_matrix= pcloud_matrix[::4]
            # pcloud_matrix_tensor = torch.tensor(pcloud_matrix,device=self.device,dtype=torch.float64)

            # # Transform
            # tf_cloud_tensor = TransCloudTensor(pcloud_matrix_tensor, device=self.device)

            # # Extract heightmap
            # sampled_heightData=key_points(tf_cloud_tensor, self.heightmap, device="cuda:0", nulls = self.nulls)
            
            # Get the pose of the left eye of the camera with reference to the world frame
            self.zed_mini.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
            self.zed_mini.get_sensors_data(self.zed_sensors, sl.TIME_REFERENCE.IMAGE)

            # Display the translation and timestamp
            pos_translation = sl.Translation()
            tx = round(self.zed_pose.get_translation(pos_translation).get()[0], 3)
            ty = round(self.zed_pose.get_translation(pos_translation).get()[1], 3)
            tz = round(self.zed_pose.get_translation(pos_translation).get()[2], 3)

            print(tx,ty,tz)

            print("IMU pose :" + str(self.zed_sensors.get_imu_data().get_pose()))
            # Total distance of the camera
            goal_dist = np.linalg.norm([tx-self.goal[0], tz-self.goal[1]])

            # Get the rotation of the camera
            rot_y = round(self.zed_pose.get_euler_angles(radian = True)[1], 3)
            direction_vector = [0, 0]
            direction_vector[0] = math.cos(rot_y) # x value
            direction_vector[1] = math.sin(rot_y) # y value

            heading_diff_ang = math.atan2(self.goal[0] * direction_vector[1] - self.goal[1] * direction_vector[0], self.goal[0] * direction_vector[0] + self.goal[1] * direction_vector[1])

            if self.debug:
                with_color = torch.tensor(np.reshape(point_cloud_data[:,:,0:4], (-1,4))[::4],device=self.device)
                torch.save(with_color, 'Original.pt')
                torch.save(tf_cloud_tensor, 'Transformed.pt')


                sampled_heightData[self.map._remove_points(),2] = 0 
                torch.save(sampled_heightData, 'Heightmap.pt')
            
            torch.cuda.synchronize()

            return goal_dist, heading_diff_ang#, sampled_heightData
            
        else:
            return 0, 0, 0


def main(args=None):
    TestTensor = torch.tensor([2,3,1,31],device="cpu")
    camera = ZedCamera(debug = False, device = "cpu", nulls = 0.0)

    print("Initialized class")
    lastrun = 0.0
    while 1:
        if time.perf_counter() > lastrun + 0.4: 
            lastrun = time.perf_counter()
            camera.callback()
    

    # for i in range(10):
    #     start = time.perf_counter()
    #     camera.callback()
    #     print("Done: ", time.perf_counter()-start)

if __name__ == '__main__':
    main()