import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import torch
import sys
from pypcd import pypcd
import pprint
import struct

def encode(v):
    enc = np.float32((1.0, 256.0, 256.0**2, 256.0**3)) * v
    enc = frac(enc)
    enc -= (enc[1], enc[2], enc[3], enc[3]) * np.float32((1/256.0, 1/256.0, 1/256.0, 0.0))
    return enc

def frac(x):
    return x - np.floor(x)

def decode(rgba):
    return np.dot(rgba, np.float32((1.0, 1/256.0, 1/(256.0**2), 1/(256.0**3))))

if __name__ == '__main__':
    # Original point cloud
    OriginalCloud = torch.load('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/camera_utils/Original.pt').numpy()
    x, y, z, rgba = OriginalCloud[::1,0], OriginalCloud[::1,1], OriginalCloud[::1,2], OriginalCloud[::1,3] 

    colors = []

    for i, color in enumerate(rgba):
        ba = (bytearray(struct.pack("f", color)))   
        hex = [ "0x%0x" % b for b in ba ]
        r = float(int(hex[0],16))/float(255)
        g = float(int(hex[1],16))/float(255)
        b = float(int(hex[2],16))/float(255)
        a = float(int(hex[3],16))/float(255)
        colors.append([r, g, b])

    # Plot
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(projection='3d')
    img = ax.scatter(x, y, z,c=colors)
    ax.axes.set_xlim3d(left=-2.5,right=2.5)
    ax.axes.set_ylim3d(bottom=-2.5,top=2.5)
    ax.axes.set_zlim3d(bottom=0,top=5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    # Transformed pointcloud
    TransformedCloud = torch.load('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/camera_utils/Transformed.pt').numpy()
    xt, yt, zt = TransformedCloud[::1,0], TransformedCloud[::1,1], TransformedCloud[::1,2] 

    print(OriginalCloud.shape)
    print(TransformedCloud.shape)
    

    # Plot
    fig2 = plt.figure(figsize=(12,12))
    ax2 = fig2.add_subplot(projection='3d')
    img2 = ax2.scatter(xt, yt, zt, c=colors)
    ax2.axes.set_xlim3d(left=-2.5,right=2.5)
    ax2.axes.set_ylim3d(bottom=-0.5,top=4.5)
    ax2.axes.set_zlim3d(bottom=-0.5,top=1)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')


    # Heightmap
    Heightmap = torch.load('/home/orin/Documents/isaac_rover_physical_2.0/src/rl_camera/camera_utils/Heightmap.pt').cpu().numpy()
    hx, hy, hz = Heightmap[:,0], Heightmap[:,1], Heightmap[:,2]

    colorsR = np.where(hz > 0.05, [1.0], [0.0])
    colorsG = np.where(hz > 0.05, [0.0], [0.0])
    colorsB = np.where(hz > 0.05, [0.0], [1.0])


    colorsR = np.where(hz < -0.05, [0.0], colorsR)
    colorsG = np.where(hz < -0.05, [1.0], colorsG)
    colorsB = np.where(hz < -0.05, [0.0], colorsB)

    color = np.transpose(np.array([colorsR, colorsG, colorsB]))
    print(np.transpose(color).shape)


    img3 = ax2.scatter(hx, hy, hz,c = color)

    # #Plot
    fig3 = plt.figure(figsize=(6,6))
    ax3 = fig3.add_subplot(projection='3d')
    img3 = ax3.scatter(hx, hy, hz,c = color)
    ax3.axes.set_xlim3d(left=-1,right=1)
    ax3.axes.set_ylim3d(bottom=-1,top=1)
    ax3.axes.set_zlim3d(bottom=-1,top=1)
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    
    # fig4 = plt.figure(figsize=(6,6))
    # ax4 = fig4.add_subplot()
    # ax4.scatter(hx, hy, c=color)
    # ax.set(xlim=(-1, 1),ylim=(-1, 1))

    plt.show()
