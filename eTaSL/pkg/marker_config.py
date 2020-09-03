from .kinect import *

def get_aruco_config():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
#     params = aruco.DetectorParameters_create()

    aruco_map = {'indy0':[ObjectMarker(11, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.07,0.12,0])),
                          ObjectMarker(12, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.12,0.12,0])),
                          ObjectMarker(13, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.12,-0.12,0])),
                          ObjectMarker(14, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.07,-0.12,0]))
                         ],
                 'panda1':[ObjectMarker(21, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.128,0.12,0])),
                          ObjectMarker(22, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.112,0.12,0])),
                          ObjectMarker(23, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.112,-0.12,0])),
                          ObjectMarker(24, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.128,-0.12,0]))
                         ],
                 'floor':[ObjectMarker(101, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.1,0.075,0])),
                          ObjectMarker(102, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.1,0.075,0])),
                          ObjectMarker(103, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.1,-0.075,0])),
                          ObjectMarker(104, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.1,-0.075,0]))
                         ],
                 'wall':[ObjectMarker(91, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.1,0.075,0])),
                          ObjectMarker(92, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.1,0.075,0])),
                          ObjectMarker(93, 0.048, SE3(Rot_zyx(0,0,np.pi), [0.1,-0.075,0])),
                          ObjectMarker(94, 0.048, SE3(Rot_zyx(0,0,np.pi), [-0.1,-0.075,0]))
                         ],
                 'box1':[ObjectMarker(111, 0.03825, SE3(Rot_zyx(0,0,np.pi), [0,0,0.025])),
                          ObjectMarker(112, 0.03825, SE3(Rot_zyx(0,0,np.pi/2), [0,0.025,0])),
                          ObjectMarker(113, 0.03825, SE3(Rot_zyx(0,0,0), [0,0,-0.025])),
                          ObjectMarker(114, 0.03825, SE3(Rot_zyx(0,0,np.pi*3/2), [0,-0.025,0])),
                          ObjectMarker(115, 0.03825, SE3(Rot_zyx(0,-np.pi/2,0), [0.025,0,0])),
                          ObjectMarker(116, 0.03825, SE3(Rot_zyx(0,np.pi/2,0), [-0.025,0,0]))
                         ],
                 'goal':[ObjectMarker(201, 0.1, SE3(Rot_zyx(0,0,np.pi), [0,0,0]))]
                }
    return aruco_map, dictionary
        
# print_markers(aruco_map, dictionary, px_size = 800, dir_img = "./markers")