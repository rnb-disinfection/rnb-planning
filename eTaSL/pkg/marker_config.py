from .sensor.marker import *
from .geometry.geometry import GEOTYPE

def get_aruco_config():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    #     params = aruco.DetectorParameters_create()

    aruco_map = {
        'indy0': MarkerSet(dtype=DetectType.ROBOT, _list=[
            ObjectMarker('indy0', 11, 0.048, [-0.0712,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 12, 0.048, [0.1212,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 13, 0.048, [0.1212,-0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 14, 0.048, [-0.0712,-0.1212,0], (np.pi,0,0))
        ]),
        'panda1':MarkerSet(dtype=DetectType.ROBOT, _list=[
            ObjectMarker('panda1', 21, 0.048, [-0.1292,0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 22, 0.048, [0.1132,0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 23, 0.048, [0.1132,-0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 24, 0.048, [-0.1292,-0.1212,0], (np.pi,0,0))
        ]),
        'floor':MarkerSet(dtype=DetectType.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(1.52,0.72,0.016),
                          _list=[
                              ObjectMarker('floor', 101, 0.05, [-0.1,0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 102, 0.05, [0.1,0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 103, 0.05, [0.1,-0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 104, 0.05, [-0.1,-0.07,0.005], (np.pi,0,0))
                          ]),
        'wall':MarkerSet(dtype=DetectType.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(3,3,0.01),
                         _list=[
                             ObjectMarker('wall', 91, 0.05, [-0.1,0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 92, 0.05, [0.1,0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 93, 0.05, [0.1,-0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 94, 0.05, [-0.1,-0.075,0], (np.pi,0,0))
                         ]),
        'box1':MarkerSet(
            dtype=DetectType.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05, 0.05,0.05), color=(0.8,0.3,0.3,1),
            _list=[
                ObjectMarker('box1', 111, 0.04, [0,0,0.025], (np.pi,0,0)),
                ObjectMarker('box1', 112, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                ObjectMarker('box1', 113, 0.04, [0,0,-0.025], (0,0,0)),
                ObjectMarker('box1', 114, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                ObjectMarker('box1', 115, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                ObjectMarker('box1', 116, 0.04, [-0.025,0,0], (0,np.pi/2,0))
            ]),
        'box2':MarkerSet(
            dtype=DetectType.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05, 0.05,0.05), color=(0.3,0.3,0.8,1),
            _list=[
                ObjectMarker('box2', 121, 0.04, [0,0,0.025], (np.pi,0,0)),
                ObjectMarker('box2', 122, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                ObjectMarker('box2', 123, 0.04, [0,0,-0.025], (0,0,0)),
                ObjectMarker('box2', 124, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                ObjectMarker('box2', 125, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                ObjectMarker('box2', 126, 0.04, [-0.025,0,0], (0,np.pi/2,0))
            ]),
        'box3':MarkerSet(
            dtype=DetectType.MOVABLE, gtype=GEOTYPE.SPHERE, dims=(0.15,0.15,0.15), color=(0.8,0.8,0.0,0.3),
            soft=True, K_col=100,
            _list=[
                ObjectMarker('box3', 131, 0.04, [0,0,0.025], (np.pi,0,0)),
                ObjectMarker('box3', 132, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                ObjectMarker('box3', 133, 0.04, [0,0,-0.025], (0,0,0)),
                ObjectMarker('box3', 134, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                ObjectMarker('box3', 135, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                ObjectMarker('box3', 136, 0.04, [-0.025,0,0], (0,np.pi/2,0))
            ]),
        'goal':MarkerSet(
            dtype=DetectType.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
            _list=[
                ObjectMarker('goal', 201, 0.1, [0,0,0.000], (np.pi,0,0))
            ])
    }
    return aruco_map, dictionary

# print_markers(aruco_map, dictionary, px_size = 800, dir_img = "./markers")