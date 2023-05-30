#! /usr/bin/env python3

import cv2
import numpy as np
from pyzbar.pyzbar import decode
# import webbrowser

class QRDetector:
    def __init__(self):
        # Camera matrix parameters from sensor_msgs/CameraInfo.msg
        self.FX = 1465.99853515625
        self.FY = 1468.26416015625
        self.PPX = 640.0
        self.PPY = 360.0

        self.fPixels = FX

        # distance between cameras in meters
        self.b = 0.004

        self.CamMatrix = np.asarray([[FX, 0.0, PPX, 0],
                                [0.0, FY, PPY, 0],
                                [0.0, 0.0, 1.0, 0]])
        
    # Use this function to create a custon camera matrix
    def bringup_camera_matrix(fx, fy, ppx, ppy):
        '''

        :param fx: focal length x
        :param fy: focal length y
        :param ppx: principal point
        :param ppy: principal point
        :return:
        '''
        camera_matrix = np.asarray([[fx, 0.0, ppx, 0],
                                    [0.0, fy, ppy, 0],
                                    [0.0, 0.0, 1.0, 0]])

        return camera_matrix

    # If depth map from ros2 topic is not provided use this
    # TODO: Provide a disparity map

    def get_coordinate(coord_2d, disparity_map):
        '''
        :param coord_2d: coord x,y in pixel coords
        :param disparity_map: the disparity map
        :return:
        '''
        depth_map = (b * fPixels) / disparity_map
        matrix = CamMatrix
        u = int(coord_2d[0])
        v = int(coord_2d[1])
        depth = depth_map[v][u]

        X = depth * (u - matrix[0][2]) / (matrix[0][0])
        Y = depth * (v - matrix[1][2]) / (matrix[1][1])
        Z = depth
        return [X, Y, Z]

    def detect(image_rgb, depth):
        for code in decode(image_rgb):
            qr_link = code.data.decode("utf-8")
            print(code.data)
            print()

            # b = webbrowser.open(qr_link)


            # Draw BBOX
            bbox_points = code.polygon
            # print(bbox_points)

            # See the how are the points and where they belong to
            points = np.array([code.polygon], np.int32)

            # Drawing the boundy box  the center of it and the position of each point
            cv2.polylines(frame, [points], True, (255, 0, 0), 3)
            cv2.putText(frame, "0", (points[0][0][0], points[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,0), 2)
            cv2.putText(frame, "1", (points[0][1][0], points[0][1][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(frame, "2", (points[0][2][0], points[0][2][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(frame, "3", (points[0][3][0], points[0][3][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            #Getting the center of the bbx

            center_bbx = [abs((points[0][1][0] + points[0][0][0])/2), abs((points[0][2][1] + points[0][1][1])/2)]
            # print(center_bbx)
            cv2.circle(frame, (int(center_bbx[0]), int(center_bbx[1])), 2, (255, 0, 0), 2)

            # Show the image
            cv2.imshow("qr", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



########################################################################################
########################################################################################
