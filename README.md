# isaac_rover_physical_2.0

## Table of content
[Old repository](#old-repository)<br/>
[Joystick control](#Joystick-control)<br/>
[Documentation](#documentation)<br/>



P7 - Rover
## Old repository
https://github.com/abmoRobotics/isaac_rover_physical


## Joystick control
The joystick used is the Logitech F710.
1. By default the robot is driven in automatic mode. Click Y to enable the joystick and set the manual mode the robot can be controlled by using the left stick.
2. Select the maximum speed by using the cross.
3. To drive the robot in any direction move the left stick and graduate the current speed using LT.
Note: The cross can be clicked at any time to change the maximum speed.
4. To make the robot rotate on itself push the left stick to the sides
5. To disable turning mode click B.
6. To enable automatic robot drive click X or A.
7. Prees LB to power off the motors (TRY TO AVOID TURNING OFF THE MOTORS WHILE DRIVING, TO NOT DAMAGE THE ROVER).

The code for the joystick can be found here: [link](https://github.com/abmoRobotics/isaac_rover_physical_2.0/blob/main/src/controller/controller/joystick.py)

## Terminal commands
Use the docker to build the entire images of ros and the dependencies, run:
```bash
docker exec -it distracted_ride bash
```

## Documentation 
### 1. Motor controllers: 
Nanotech controllers C5-E-2-09: [link to motor controllers](https://en.nanotec.com/products/1768-c5-e-2-09-motor-controller-drive-for-canopen-or-usb)

The CAN bus is set up with the following guide to controle the motors: [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9). Remember that to check if the GPIO on the Xavior is the same as on the Orin, there might be some small adjustments. Doc to [34.1.1](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/index.html) and to [35.1 GA](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/index.html).

- All the motors is setup useing the Nanotech Autosetup, and should NOT be changed! If doing autosetup, REMEMBER to do it without load.  

- The code for controlling the motors can be forund here: [motor.py](https://github.com/abmoRobotics/isaac_rover_physical_2.0/blob/main/src/controller/controller/motor.py) the way the motors is controlled is thru SDO, other solutions do exist.

- Kinematic ( Ackermann steering) [Kinematics.py](https://github.com/abmoRobotics/isaac_rover_physical_2.0/blob/main/src/controller/controller_utils/kinematics.py)

### 2. NVIDIA Jetson AGX Orin Developer Kit

The NVIDIA ORIN has some minor things that do not work: 

1. DO NOT CONNECT A DP TO HDMI CONVERTET TO MONITOR UNTIL YOU HAVE READ THIS!!! [Orin AGX Display not working](https://forums.developer.nvidia.com/t/orin-agx-display-not-working/214141/6)  

2. The ZED2i camera is not well integrated into the code, so the cameras PointCloud might be slow when processing high resolution. 

