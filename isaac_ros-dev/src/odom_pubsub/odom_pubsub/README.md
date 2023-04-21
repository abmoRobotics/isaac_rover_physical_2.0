To get access to the output of /odom :

``bash
ros2 run odom_pubsub odometry
```
Check the output with 

``bash
ros2 topic echo /odom
```


To get access to the output of /imu :

``bash
ros2 run odom_pubsub imu
```
Check the output with 

``bash
ros2 topic echo /imu
```