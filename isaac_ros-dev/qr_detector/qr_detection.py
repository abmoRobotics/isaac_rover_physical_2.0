import cv2
import numpy as np
from pyzbar.pyzbar import decode
# import webbrowser

# Camera matrix parameters from sensor_msgs/CameraInfo.msg
FX = 1465.99853515625
FY = 1468.26416015625
PPX = 640.0
PPY = 360.0

fPixels = FX

# distance between cameras in meters
b = 0.004

CamMatrix = np.asarray([[FX, 0.0, PPX, 0],
            		    [0.0, FY, PPY, 0],
             		    [0.0, 0.0, 1.0, 0]])
# Create camera matrix


# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.


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




# initialize the cv2 QRCode detector
detector = cv2.QRCodeDetector()
# qr_code = cv2.imread("Google.png")
detected = False
# cv2.imshow("qr", qr_code)


# Detect and decode the QR code

cap = cv2.VideoCapture(0)
i = 0
while True:
    ret, frame = cap.read()

    # print(decode(frame))

    # Try to open the QR message
    # Can detect multiple QR-codes, a way to solve labeling/classification
    # can be done by providing different messaages and if statements
    if detected == False:

        for code in decode(frame):
            qr_link = code.data.decode("utf-8")
            print(code.data)
            print()

            # b = webbrowser.open(qr_link)


            # Draw BBOX
            bbox_points = code.polygon
            # print(bbox_points)

            # See the how are the points and where they belong to
            points = np.array([code.polygon], np.int32)
            # print("points", points)
            # print("POINTS 0.0 ", points[0][0][0])
            # print("POINTS 0.1 ", points[0][0][1])
            # print("POINTS 1.0 ", points[0][1][0])
            # print("POINTS 1.1 ", points[0][1][1])
            # print("POINTS 2.0 ", points[0][2][0])
            # print("POINTS 2.1 ", points[0][2][1])
            # print("POINTS 3.0 ", points[0][3][0])
            # print("POINTS 3.1 ", points[0][3][1])
            # print(' ')
            # print(print("POINTS 1", points[1]))

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

            # Stop detection
            # detected = True

    # else:
    #     break

    # Save img
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite(f"ss{i}.jpg", frame)
        i+=1


    # Show the image
    cv2.imshow("qr", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # See what the detector provides
    # print('data', data)
    # print()
    # print('bbox', bbox)
    # print()
    # print("_", _)

# cv2.waitKey(0)
cv2.destroyAllWindows()

