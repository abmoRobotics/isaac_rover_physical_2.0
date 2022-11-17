import os
import numpy as np
import canopen
from canopen import *
import Jetson.GPIO as GPIO 

def autosetup(ID):
    with open('/home/orin/Documents/isaac_rover_physical_2.0/src/controller/config/odscript.txt') as update_file:
        lines = [line.rstrip('\n') for line in update_file]
    eds='/home/orin/Documents/isaac_rover_physical_2.0/src/controller/config/C5-E-2-09.eds'
    GPIO.setmode(GPIO.BOARD)
    index=[]
    subindex=[]
    value=[]
    for i in range(0,len(lines)):
        index=np.append(index,lines[i].replace('=',':').replace('//',':').split(':')[0])
        subindex=np.append(subindex,lines[i].replace('=',':').replace('//',':').split(':')[1])
        value=np.append(value,lines[i].replace('=',':').replace('//',':').split(':')[2])
    network=canopen.Network()
    network.connect(channel='can1',bustype='socketcan')
    for id in ID:
        print(id)
        node=network.add_node(id+1, eds)
        for i in range(len(index)):
            l=int(index[i],base=16)
            m=int(subindex[i],base=16)
            n=int(value[i])
            if m==0:
                node.sdo[l].phys = n
            else:
                node.sdo[l][m].phys = n