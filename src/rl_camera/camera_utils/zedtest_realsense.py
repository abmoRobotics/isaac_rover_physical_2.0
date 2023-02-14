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
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation



class ZedCamera:
    def __init__(self, debug = False, device = "cuda:0", nulls = -3.0, goal = [-3, 3]):

        self.debug = debug
        self.device = device

        self.map=Heightmap("cuda:0")
        self.heightmap = self.map.point_distribution
        self.heightmap[:,0] = -self.heightmap[:,0]
        self.nulls = nulls

        # Camera serial numbers
        zed_2_serial    = 37915676
 
        # Create a InitParameters object for ZED2 and set configuration parameters
        init_params = sl.InitParameters()
        init_params.set_from_serial_number(zed_2_serial)
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
        #init_params.coordinate_system = sl.COORDINATE_SYSTEM_IMAGE
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.VGA
        init_params.depth_stabilization = False
        init_params.depth_minimum_distance = 0.15
        init_params.depth_maximum_distance = 5.0

        # Create a Camera object for ZED2 and open
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("There was an error opening the ZED camera. Is it connected?")
            exit(1)
        
        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100
       # self.runtime_parameters.remove_saturated_areas = True

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        # Build config object and request pose data
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        # Start streaming with requested config
        self.pipe.start(cfg)

        # Initialize image and depth
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.data_structure = []

        # Initilize for camera position tracker
        self.goal = goal
        self.tx_old = 0
        self.ty_old = 0
        self.zed_pose = sl.Pose()
        self.zed_sensors = sl.SensorsData()
        initial_trans = sl.Transform()
        initial_trans.set_euler_angles(0,0,0,radian=False)  
        self.dist_total = 0

    def callback(self):

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
            sampled_heightData=key_points(tf_cloud_tensor, self.heightmap, device="cuda:0", nulls= self.nulls)
            

            # Wait for the next set of frames from the camera
            frames = self.pipe.wait_for_frames()

            # Fetch pose frame
            pose = frames.get_pose_frame()
            data = pose.get_pose_data()

            # Get the translation
            tx = -round(data.translation.z, 3)
            ty = -round(data.translation.x, 3)

            x_move = abs(self.tx_old - tx)
            y_move = abs(self.ty_old - ty)

            dist = math.sqrt((x_move*x_move)+(y_move*y_move))

            self.dist_total = self.dist_total + dist

            self.tx_old = tx
            self.ty_old = ty

            # Get rotation
            w = data.rotation.w
            x = data.rotation.z
            y = data.rotation.x
            z = data.rotation.y

            x, y, z = self.quaternion_to_euler_angle(w, x, y, z)
            rot_y = round((z*math.pi)/180, 3)

            direction_vector = [0, 0]
            direction_vector[0] = math.cos(rot_y) # x value
            direction_vector[1] = math.sin(rot_y) # y value

            rover_dir = math.atan2(direction_vector[1], direction_vector[0])


            rover_to_goal_vector = [self.goal[0] - tx, self.goal[1] - ty]
            goal_dir = math.atan2(rover_to_goal_vector[1], rover_to_goal_vector[0])

            rover_to_goal_angle = (goal_dir-rover_dir)

            rover_to_goal_angle = np.where(rover_to_goal_angle < -3.14, rover_to_goal_angle + math.pi*2, rover_to_goal_angle)
            rover_to_goal_angle = np.where(rover_to_goal_angle > 3.14, rover_to_goal_angle - math.pi*2, rover_to_goal_angle)
            
            # Total distance of the camera
            goal_dist = np.linalg.norm(rover_to_goal_vector)

            print("location: ", tx,ty)
            print("dist: ", goal_dist, " ang: ", rover_to_goal_angle)

            if self.debug:
                with_color = torch.tensor(np.reshape(point_cloud_data[:,:,0:4], (-1,4))[::4],device=self.device)
                torch.save(with_color, 'Original.pt')
                torch.save(tf_cloud_tensor, 'Transformed.pt')


                sampled_heightData[self.map._remove_points(),2] = 0 
                torch.save(sampled_heightData, 'Heightmap.pt')
            
            torch.cuda.synchronize()

            return tx, ty, self.dist_total, goal_dist, rover_to_goal_angle, sampled_heightData, image_ocv
            
        else:
            return 0, 0, 0
    
    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

def main(args=None):
    TestTensor = torch.tensor([2,3,1,31],device="cpu")
    camera = ZedCamera(debug = False, device = "cpu", goal = [20, 0], nulls=-2.0)

    print("Initialized class")
    camera.callback()

    # lastrun = 0.0
    # while 1:
    #     if time.perf_counter() > lastrun + 0.2: 
    #         lastrun = time.perf_counter()
    #         camera.callback()
    #         print("Running")

    

    # for i in range(10):
    #     start = time.perf_counter()
    #     camera.callback()
    #     print("Done: ", time.perf_counter()-start)

if __name__ == '__main__':
    main()