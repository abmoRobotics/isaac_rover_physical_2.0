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
import sys
import os
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header
from heightmap_distribution import Heightmap


   
def key_points(heightData, heightmap, device = 'cpu', nulls = -2.0):
    start = torch.cuda.Event(enable_timing=True)
    end = torch.cuda.Event(enable_timing=True)
 
    device = device
    data = heightData.to(device)

    # Height data
    dataZ = data[:, 2]

    # Empty return vector
    sampled_heightData = torch.zeros(heightmap.shape[0], device=device)

    # Expand data to heightmaps size
    data = data.view(data.shape[0],data.shape[1],-1).expand([data.shape[0],data.shape[1], heightmap.shape[0]])
    offset = 0.05
    
    # Amount of points to treat at once. Decreasing lowers ram usage.
    k = heightmap.shape[0]
    
    # Tool tensors
    ones = torch.ones_like(data[:,2, 0:k], dtype=torch.uint8, device=device)
    zeros = torch.zeros_like(data[:,2, 0:k], dtype=torch.uint8, device=device)

    for i in range((int)(heightmap.shape[0]/k)):
        
        # If conditions check if point is within offset.
        belong = torch.zeros((data.shape[0], k), device=device)
        belong = torch.where((data[:,0, i*k:i*k+k] > heightmap[i*k:i*k+k,0]-offset) & (data[:,0, i*k:i*k+k] < heightmap[i*k:i*k+k,0]+offset) & (data[:,1, i*k:i*k+k] > heightmap[i*k:i*k+k,1]-offset) & (data[:,1, i*k:i*k+k] < heightmap[i*k:i*k+k,1]+offset), ones, zeros )

        dataZexpanded = dataZ.view(data.shape[0],-1).expand([data.shape[0], k])
        
        # Filter out points outside heightmap
        dataFiltered = dataZexpanded * belong
        dataFiltered = torch.where(dataFiltered == 0, torch.ones_like(dataFiltered)*-3, dataFiltered)        
        dataFiltered = torch.nan_to_num(dataFiltered,nan = -3.0)

        
        # Return the heighest point within the area
        sampled_heightData[i*k:i*k+k] = torch.amax(dataFiltered, 0)
        
        torch.cuda.synchronize()
        
    sampled_heightData = torch.nan_to_num(sampled_heightData, nan=nulls)
    sampled_heightData = torch.where(sampled_heightData < -2.9, torch.zeros_like(sampled_heightData), sampled_heightData)
    #sampled_heightData = torch.ones_like(sampled_heightData)*-1
    sampled_heightData = torch.stack((heightmap[:,0], heightmap[:,1], sampled_heightData[:])).T

    return sampled_heightData
    
def TransCloudTensor(pointCloud, device="cpu"):

    x_rot = (90+40) #placed in 61 degrees than 29+90 =119, 61+90=151
    y_rot = 0 #Guessed
    z_rot = 0 #Guessed
    tx = 0.00 # unsure of the unit
    ty = -0.15128 # 1.384# 0.15128
    tz = 1.00306 #1.250# 1.00306

    omega = math.radians(x_rot)
    theta = math.radians(y_rot)
    kappa = math.radians(z_rot)

    ###############################################################
    # Rotation and translation matrices

    rotMat_x = np.array([   [1, 0, 0, 0],
                            [0, math.cos(omega), -(math.sin(omega)), 0],
                            [0, math.sin(omega), math.cos(omega), 0],
                            [0, 0, 0, 1]    
                        ])
    rotMat_y = np.array([   [math.cos(theta), 0, math.sin(theta), 0],
                            [0, 1, 0, 0],
                            [-(math.sin(theta)), 0, math.cos(theta), 0],
                            [0, 0, 0, 1]    
                        ])
    rotMat_z = np.array([   [math.cos(kappa), -(math.sin(kappa)), 0, 0],
                            [math.sin(kappa), math.cos(kappa), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]    
                        ])

    projection_mat = np.matmul(rotMat_x, rotMat_y)
    projection_mat = np.matmul(projection_mat, rotMat_z)
    projection_mat_tensor = torch.tensor(projection_mat[:3,:3], device=device)
    tf_cloud = torch.matmul(pointCloud,projection_mat_tensor)  
        
    tf_cloud[:,0] += tx
    tf_cloud[:,1] += ty
    tf_cloud[:,2] += tz

    return tf_cloud

def TransCloud(pointCloud):

    x_rot = (90+40) #placed in 61 degrees than 29+90 =119, 61+90=151
    y_rot = 0 #Guessed
    z_rot = 0 #Guessed
    tx = 0 # unsure of the unit
    ty = 0.15128
    tz = 1.00306

    omega = math.radians(x_rot)
    theta = math.radians(y_rot)
    kappa = math.radians(z_rot)

    

    ###############################################################
    # Rotation and translation matrices

    rotMat_x = np.array([   [1, 0, 0, 0],
                            [0, math.cos(omega), -(math.sin(omega)), 0],
                            [0, math.sin(omega), math.cos(omega), 0],
                            [0, 0, 0, 1]    
                        ])
    rotMat_y = np.array([   [math.cos(theta), 0, math.sin(theta), 0],
                            [0, 1, 0, 0],
                            [-(math.sin(theta)), 0, math.cos(theta), 0],
                            [0, 0, 0, 1]    
                        ])
    rotMat_z = np.array([   [math.cos(kappa), -(math.sin(kappa)), 0, 0],
                            [math.sin(kappa), math.cos(kappa), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]    
                        ])

    projection_mat = np.matmul(rotMat_x, rotMat_y)
    projection_mat = np.matmul(projection_mat, rotMat_z)
    tf_cloud = np.matmul(pointCloud,projection_mat[:3,:3])  
    #tf_cloud = np.delete(tf_cloud, 3, 1)

    tf_cloud[:,0] += tx
    tf_cloud[:,1] += ty
    tf_cloud[:,2] += tz
    #tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] > -0.15), axis=0) # Remove points closer than 0.2 meters of the robot
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] < -1.2), axis=0)
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] > 1.2), axis=0)
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] < -1.2), axis=0)

    return tf_cloud

    

def limit_x(x):
    value_limitx = x*(4.3315)-0.129945
    return value_limitx


def point_cloud(points, parent_frame):
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame)

    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )