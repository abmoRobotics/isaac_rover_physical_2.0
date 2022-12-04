#!/usr/bin/env python3
# CameraFunctions.py
from logging.handlers import DEFAULT_SOAP_LOGGING_PORT
#import pyrealsense2 as rs
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import time
import torch
import csv


class Cameras:
    def __init__(self):
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline() #Make a pipeline for the T265
        self.doLogging = False #Flag whether to log positions and motor values or not
        if self.doLogging:
            f = open('/home/xavier/isaac_rover_physical/exomy/scripts/utils/csv/Position.csv', 'w')
            self.writer = csv.writer(f)
        # Build config object and request pose data
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose) #Configure stream for pose data 

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.heightmap_distribution = torch.tensor(self.heightmap_distribution( 1.12, 1.2, square=True, y_start=0.03, delta=0.05, front_heavy=0.0), device='cuda:0')
        for x in range(5):
            self.pipe.wait_for_frames() #pipe is to transfer from different methods

    def callback(self, pointcloud, pointcloud2):
        start = time.perf_counter()
        frames = self.pipe.wait_for_frames()
        
        # Fetch pose frame
        pose = frames.get_pose_frame()
        data = pose.get_pose_data()
        
        pos = data.translation
        vel = data.velocity
        acc = data.acceleration
        rot = data.rotation
        ang_vel = data.angular_velocity
        ang_acc = data.angular_acceleration
        #Transform points based on the rotation of the T265
        RobotPos = self.TransPoint(np.array([pos.x, pos.y, pos.z]))
        RobotVel = self.TransPoint(np.array([vel.x, vel.y, vel.z]))
        RobotAcc = self.TransPoint(np.array([acc.x, acc.y, acc.z]))
        r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        Rot_vec = r.as_rotvec()
        RobotRot = self.TransPoint(np.array([Rot_vec[0], Rot_vec[1], Rot_vec[2]]))
        if self.doLogging:
            self.writer.writerow(RobotPos)

        #Transform and combine pointcloud data 
        tf_cloud = self.TransCloud(pointcloud)
        tf_cloud2 = self.TransCloud(pointcloud2)
        tf_cloudSum = np.append(tf_cloud, tf_cloud2, axis=0)
        start_to_tensor = time.perf_counter()
        points = self.key_points(torch.tensor(tf_cloudSum))   
        end_to_tensor = time.perf_counter() - start_to_tensor
                    
                 
        #elaps = time.perf_counter() - start
        elaps = end_to_tensor
        return tf_cloudSum, RobotPos, RobotVel, RobotAcc, RobotRot, ang_vel, ang_acc, points, elaps


    # def euler_from_quaternion(self, x, y, z, w):
    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw)
    #     roll is rotation around x in radians (counterclockwise)
    #     pitch is rotation around y in radians (counterclockwise)
    #     yaw is rotation around z in radians (counterclockwise)
    #     """
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll_x = math.atan2(t0, t1)
     
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch_y = math.asin(t2)
     
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw_z = math.atan2(t3, t4)
     
    #     return roll_x, pitch_y, yaw_z # in radians 

    def TransPoint(self, point):
        x_rot = -11-90
        y_rot = 180 
        z_rot = 0 

        omega = math.radians(x_rot)
        theta = math.radians(y_rot)
        kappa = math.radians(z_rot)

       

        ###############################################################
        # Rotation and translation matrices
        point = np.append(point, 1)
        rotMat_x = np.array([   [1, 0, 0, 0],
                                [0, math.cos(omega), math.sin(omega), 0],
                                [0, -(math.sin(omega)), math.cos(omega), 0],
                                [0, 0, 0, 1]    
                            ])
        rotMat_y = np.array([   [math.cos(theta), 0, -(math.sin(theta)), 0],
                                [0, 1, 0, 0],
                                [math.sin(theta), 0, math.cos(theta), 0],
                                [0, 0, 0, 1]    
                            ])
        rotMat_z = np.array([   [math.cos(kappa), math.sin(kappa), 0, 0],
                                [-(math.sin(kappa)), math.cos(kappa), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]    
                            ])
        projection_mat = np.matmul(np.matmul(rotMat_x, rotMat_y), rotMat_z)
        tf_point = np.matmul(point,projection_mat)  
        tf_point = np.delete(tf_point, 3)
        return tf_point


    def TransCloud(self, pointCloud, cam):
        if cam == 1:
            self.x_rot = 90 #placed in 61 degrees than 29+90 =119, 61+90=151
            self.y_rot = 0 #Guessed
            self.z_rot = 180 #Guessed
            self.tx = -0.04445 # unsure of the unit
            self.ty = -0.04809 # 
            self.tz = 0.39438 # 
        elif cam == 2:
            self.x_rot = -26.7-90 #placed in 61 degrees than 29+90 =119, 61+90=151
            self.y_rot = 0 #Guessed
            self.z_rot = 135 #Guessed
            self.tx = 0.07295 # unsure of the unit
            self.ty = -0.02622 # 
            self.tz = 0.39438 # 

        omega = math.radians(self.x_rot)
        theta = math.radians(self.y_rot)
        kappa = math.radians(self.z_rot)

        

        ###############################################################
        # Rotation and translation matrices

        rotMat_x = np.array([   [1, 0, 0, 0],
                                [0, math.cos(omega), math.sin(omega), 0],
                                [0, -(math.sin(omega)), math.cos(omega), 0],
                                [0, 0, 0, 1]    
                            ])
        rotMat_y = np.array([   [math.cos(theta), 0, -(math.sin(theta)), 0],
                                [0, 1, 0, 0],
                                [math.sin(theta), 0, math.cos(theta), 0],
                                [0, 0, 0, 1]    
                            ])
        rotMat_z = np.array([   [math.cos(kappa), math.sin(kappa), 0, 0],
                                [-(math.sin(kappa)), math.cos(kappa), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]    
                            ])

        projection_mat = np.matmul(rotMat_x, rotMat_y)
        projection_mat = np.matmul(projection_mat, rotMat_z)
        tf_cloud = np.matmul(pointCloud,projection_mat)  
        tf_cloud = np.delete(tf_cloud, 3, 1)
       
        tf_cloud[:,0] += self.tx
        tf_cloud[:,1] += self.ty
        tf_cloud[:,2] += self.tz
        #tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] > -0.15), axis=0) # Remove points closer than 0.2 meters of the robot
        tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] < -1.2), axis=0)
        tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] > 1.2), axis=0)
        tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] < -1.2), axis=0)

        return tf_cloud


    def heightmap_distribution(x_limit, y_limit, square=False, y_start=0.296, delta=0, front_heavy=0):

        point_distribution = []

        # If delta variable not set, exit.
        if delta == 0:
            print("Need delta value!")
            exit()

        xd = 0
        yd = 0

        y = y_start
        while y < y_limit:
            
            x = 0

            delta += front_heavy

            flag = True
            if square==False:
                limit = limit_x(y)
                if x_limit < limit_x(y):
                    limit = x_limit
            else:
                limit = x_limit


            while x < limit:
                
                if x < -limit:
                    x += delta
                    xd += 1
                    flag = False

                if flag:
                    x -= delta
                    xd -= 1
                else:
                    point_distribution.append([x, -y])
                    x += delta
                    xd += 1

            y += delta
            yd +=1

        point_distribution = np.round(point_distribution, 4)



        return point_distribution

        

    def limit_x(x):
        return x*(4.3315)-0.129945

    
    def key_points(heightData):
        device = torch.device('cuda:0')
        data = heightData.to(device)
        data = data[0::2]
        heightmap = heightmap_distribution(20,20,square=False, y_start=0.296, delta=1, front_heavy=0)
        # Height data
        dataZ = data[:, 2]
        
        sampled_heightData = torch.zeros(heightmap.shape[0], device=device)

        # Expand data to heightmaps size
        data = data.view(data.shape[0],data.shape[1],-1).expand([data.shape[0],data.shape[1], heightmap.shape[0]])
        offset = 0.05
        k = 108
        ones = torch.ones_like(data[:,2, 0:k], dtype=torch.uint8, device=device)
        zeros = torch.zeros_like(data[:,2, 0:k], dtype=torch.uint8, device=device)

        for i in range((int)(heightmap.shape[0]/k)):
            belong = torch.zeros((data.shape[0], k), device=device)

            AboveX = torch.zeros((data.shape[0], k), device='cuda:0')
            UnderX = torch.zeros((data.shape[0], k), device='cuda:0')
            AboveY = torch.zeros((data.shape[0], k), device='cuda:0')
            UnderY = torch.zeros((data.shape[0], k), device='cuda:0')

            AboveX[:] = torch.where(data[:,0, i*k:i*k+k] > heightmap[i*k:i*k+k,0]-offset, ones, zeros )
            UnderX[:] = torch.where(data[:,0, i*k:i*k+k] < heightmap[i*k:i*k+k,0]+offset, ones, zeros )
            AboveY[:] = torch.where(data[:,1, i*k:i*k+k] > heightmap[i*k:i*k+k,1]-offset, ones, zeros )
            UnderY[:] = torch.where(data[:,1, i*k:i*k+k] < heightmap[i*k:i*k+k,1]+offset, ones, zeros )
            X_cond = torch.bitwise_and(AboveX==1, UnderX==1)
            Y_cond = torch.bitwise_and(AboveY==1, UnderY==1)
            belong = torch.where(torch.bitwise_and(X_cond, Y_cond), ones, zeros)

            dataZexpanded = dataZ.view(data.shape[0],-1).expand([data.shape[0], k])

            # Filter out points outside heightmap
            dataFiltered = dataZexpanded * belong
            #dataFiltered = torch.where(dataFiltered == 0, torch.zeros_like(dataFiltered)-3, dataFiltered)
            # Sort data
            #dataSorted, indx1 = torch.sort(dataFiltered,0, descending=True)
            #belongsSorted, indx2 = torch.sort(belong,0, descending=True)

            # Find Average of top 15 measurements
            #sum = torch.sum(dataSorted[0:15,:], 0)
            #Index = torch.sum(belongsSorted[0:15,:], 0)
            sampled_heightData[i*k:i*k+k] = torch.amax(dataFiltered, 0)

        sampled_heightData = torch.nan_to_num(sampled_heightData)
        #sampled_heightData = torch.where(sampled_heightData == -3, torch.zeros_like(sampled_heightData), sampled_heightData)
        sampled_heightData = torch.stack((heightmap[:,0], heightmap[:,1], sampled_heightData[:])).T

        return sampled_heightData
    
