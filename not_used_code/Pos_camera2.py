#!/usr/bin/env python3
import pyzed.sl as sl
from scipy.spatial.transform import Rotation as R


def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    initial_trans = sl.Transform()
    
    initial_rot = sl.Rotation()
    initial_rot.set_rotation_vector(0,1,0)
    initial_trans.setRotationMatrix(initial_rot)
    #initial_rot.identity()
    #initial_rot.set_rotation()
    #initial_trans.set_rotation_vector(2,0,0)
    #initial_trans.set_euler_angles(0,0,0,radian = False )
    

    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)
    # Use a right-handed Y-up coordinate system
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Enable positional tracking with default parameters
    py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
    tracking_parameters = sl.PositionalTrackingParameters(initial_trans)
    #tracking_parameters.set_initial_world_transform(initial_trans)
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)


    # As long as camera is open, it will track the position
    zed_pose = sl.Pose()

    zed_sensors = sl.SensorsData()
    runtime_parameters = sl.RuntimeParameters()

    while sl.ERROR_CODE.SUCCESS:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Get the pose of the left eye of the camera with reference to the world frame
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
            print(zed_pose.get_rotation_vector())
            zed_imu = zed_sensors.get_imu_data()

            # Display the translation and timestamp
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, zed_pose.timestamp.get_milliseconds()))

            x_rot = -36.81294151251732
            y_rot = -1.3037060699855711
            z_rot = -1.7388430777330455
            
            #pos_rot = sl.Rotation()
            rot_x = round(zed_pose.get_euler_angles(radian = False)[0], 3)
            rot_y = round(zed_pose.get_euler_angles(radian = False)[1], 3)
            rot_z = round(zed_pose.get_euler_angles(radian = False)[2], 3)
            print("Rotation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(rot_x, rot_y, rot_z, zed_pose.timestamp.get_milliseconds()))

        # Close the camera
        zed.close()

if __name__ == "__main__":
    main()        
    

