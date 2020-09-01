import time
from pkg.kinect import *

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
params = aruco.DetectorParameters_create()

aruco_map = {'indy0':[ObjectMarker(11, 0.05, SE3(Rot_zyx(0,0,0), [-0.075,0.125,0])),
                      ObjectMarker(12, 0.05, SE3(Rot_zyx(0,0,0), [0.125,0.125,0])),
                      ObjectMarker(13, 0.05, SE3(Rot_zyx(0,0,0), [0.125,-0.125,0])),
                      ObjectMarker(14, 0.05, SE3(Rot_zyx(0,0,0), [-0.075,-0.125,0]))
                     ]}


pyK4A = init_kinect()
time.sleep(1)
cameraMatrix, distCoeffs = get_calibration(pyK4A)

while True:
    color_image = get_image_set(pyK4A)
    if color_image is None:
        time.sleep(100e-3)
        print('get_image error')
        continue
    print('got image')
    color_image = color_image.copy()
    print('copy')
    objectPose_dict, corner_dict = get_object_pose_dict(color_image, aruco_map, dictionary, cameraMatrix, distCoeffs)
    print('pose')
    color_image_out = draw_objects(color_image, aruco_map, objectPose_dict, corner_dict, cameraMatrix, distCoeffs, axis_len=0.1)
    print('draw')
    cv2.imshow('image', cv2.resize(color_image_out, None, fx=0.5, fy=0.5))
    print('imshow')
    if cv2.waitKey(50) == 27:
        break
    
cv2.destroyAllWindows()
disconnect_kinect(pyK4A)