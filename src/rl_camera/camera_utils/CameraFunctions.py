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


   
def key_points(heightData):
    device = torch.device('cuda:0')
    data = heightData.to(device)
    data = data[0::2]
    #map=Heightmap(device)
    #heightmap = map.point_distribution
    heightmap = heightmap_distribution()
    # Height data
    dataZ = data[:, 2]

    sampled_heightData = torch.zeros(heightmap.shape[0], device=device)

    # Expand data to heightmaps size
    data = data.view(data.shape[0],data.shape[1],-1).expand([data.shape[0],data.shape[1], heightmap.shape[0]])
    # print(data.shape)
    offset = 0.05#0.05
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
        dataFiltered=torch.nan_to_num(dataFiltered)
        dataFiltered = torch.where(dataFiltered==torch.zeros_like(dataFiltered),torch.ones_like(dataFiltered)*-3,dataFiltered)
        sampled_heightData[i*k:i*k+k] = torch.amax(dataFiltered, 0)
    

    sampled_heightData = torch.nan_to_num(sampled_heightData)
    #sampled_heightData = torch.where(sampled_heightData == -3, torch.zeros_like(sampled_heightData), sampled_heightData)
    sampled_heightData = torch.stack((heightmap[:,0], heightmap[:,1], sampled_heightData[:])).T
    print("sampled heightdata")
    print(sampled_heightData)
    return sampled_heightData

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
    tf_cloud = np.matmul(pointCloud,projection_mat)  
    #tf_cloud = np.delete(tf_cloud, 3, 1)

    tf_cloud[:,0] += tx
    tf_cloud[:,1] += ty
    tf_cloud[:,2] += tz
    #tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] > -0.15), axis=0) # Remove points closer than 0.2 meters of the robot
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,1] < -1.2), axis=0)
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] > 1.2), axis=0)
    # tf_cloud = np.delete(tf_cloud, np.where(tf_cloud[:,0] < -1.2), axis=0)

    return tf_cloud

def heightmap_distribution():

    point_distribution = []

    # If delta variable not set, exit.
    jump=np.arange(-3,3,0.1).tolist()
    for y in jump:
        for x in jump:
            point_distribution.append([x,y])
    

    point_distribution = np.round(point_distribution, 4)
    point_distribution = torch.tensor(point_distribution,device="cuda:0")

    point_distribution2=point_distribution.cpu().numpy()
    np.save("/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/rl_camera/plane",point_distribution2)
    return point_distribution

    

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