from .detector import *
from ...geometry.geometry import GEOTYPE

def get_aruco_map():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    #     params = aruco.DetectorParameters_create()

    aruco_map = ArucoMap(dictionary=dictionary, _dict={
        'vive_ref':MarkerSet('vive_ref', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.6,0.4,0.01),
                          color=(0.9,0.9,0.9,0.2),
                          _list=[
                              ObjectMarker('vive_ref', 1, 0.05, [0.1,0.0,0.05], (-np.pi/2,0,np.pi)),
                              ObjectMarker('vive_ref', 2, 0.05, [-0.1,0.0,0.05], (-np.pi/2,0,np.pi)),
                              ObjectMarker('vive_ref', 3, 0.05, [-0.1,0.0,-0.05], (-np.pi/2,0,np.pi)),
                              ObjectMarker('vive_ref', 4, 0.05, [0.1,0.0,-0.05], (-np.pi/2,0,np.pi))
                          ]),
        'indy0': MarkerSet('indy0', dlevel=DetectionLevel.ROBOT, _list=[
            ObjectMarker('indy0', 11, 0.048, [-0.0712,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 12, 0.048, [0.1212,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 13, 0.048, [0.1212,-0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy0', 14, 0.048, [-0.0712,-0.1212,0], (np.pi,0,0))
        ]),
        'panda1':MarkerSet('panda1', dlevel=DetectionLevel.ROBOT, _list=[
            ObjectMarker('panda1', 21, 0.048, [-0.1292,0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 22, 0.048, [0.1132,0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 23, 0.048, [0.1132,-0.1212,0], (np.pi,0,0)),
            ObjectMarker('panda1', 24, 0.048, [-0.1292,-0.1212,0], (np.pi,0,0))
        ]),
        'indy1': MarkerSet('indy1', dlevel=DetectionLevel.ROBOT, _list=[
            ObjectMarker('indy1', 31, 0.048, [-0.0712,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy1', 32, 0.048, [0.1212,0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy1', 33, 0.048, [0.1212,-0.1212,0], (np.pi,0,0)),
            ObjectMarker('indy1', 34, 0.048, [-0.0712,-0.1212,0], (np.pi,0,0))
        ]),
        'floor':MarkerSet('floor', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(1.52,0.72,0.01),
                          _list=[
                              ObjectMarker('floor', 101, 0.05, [0.66-0.25-0.1,-0.29+0.17+0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 102, 0.05, [0.66-0.25+0.1,-0.29+0.17+0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 103, 0.05, [0.66-0.25+0.1,-0.29+0.17-0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 104, 0.05, [0.66-0.25+-0.1,-0.29+0.17-0.07,0.005], (np.pi,0,0)),
                              ObjectMarker('floor', 105, 0.05, [-0.11619042,  0.30448973,0.005], (np.pi,0,1.54217994)),
                              ObjectMarker('floor', 106, 0.05, [-0.34293798, -0.1753922,0.005], (np.pi,0,1.55579808))
                          ]),
        'wall':MarkerSet('wall', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(3,3,0.10),
                         _list=[
                             ObjectMarker('wall', 91, 0.05, [-0.1,0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 92, 0.05, [0.1,0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 93, 0.05, [0.1,-0.075,0], (np.pi,0,0)),
                             ObjectMarker('wall', 94, 0.05, [-0.1,-0.075,0], (np.pi,0,0))
                         ]),
        'track':MarkerSet('track', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.6,0.4,0.01),
                          color=(0.9,0.9,0.9,0.2),
                          _list=[
                              ObjectMarker('track', 71, 0.05, [-0.26+0.0,+0.16+0.00,0.005], (np.pi,0,0)),
                              ObjectMarker('track', 72, 0.05, [-0.26+0.2,+0.16+0.00,0.005], (np.pi,0,0)),
                              ObjectMarker('track', 73, 0.05, [-0.26+0.2,+0.16-0.09,0.005], (np.pi,0,0)),
                              ObjectMarker('track', 74, 0.05, [-0.26+0.0,+0.16-0.09,0.005], (np.pi,0,0))
                          ]),
        'curve_base':MarkerSet('curve_base', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.36,0.31,0.01),
                          color=(0.9,0.9,0.9,0.2),
                          _list=[
                              ObjectMarker('curve_base', 61, 0.05, [0.155-0.2,0.13-0.00,0.005], (np.pi,0,0)),
                              ObjectMarker('curve_base', 62, 0.05, [0.155-0.0,0.13-0.00,0.005], (np.pi,0,0)),
                              ObjectMarker('curve_base', 63, 0.05, [0.155-0.0,0.13-0.09,0.005], (np.pi,0,0)),
                              ObjectMarker('curve_base', 64, 0.05, [0.155-0.2,0.13-0.09,0.005], (np.pi,0,0))
                          ]),
        'door':MarkerSet('door', dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.36,0.3,0.01), color=(1,1,1,1),
                         _list=[
                             ObjectMarker('door', 81, 0.05, [-0.02-0.1,0.07,0], (np.pi,0,0)),
                             ObjectMarker('door', 82, 0.05, [-0.02+0.1,0.07,0], (np.pi,0,0)),
                             ObjectMarker('door', 83, 0.05, [-0.02+0.1,-0.07,0], (np.pi,0,0)),
                             ObjectMarker('door', 84, 0.05, [-0.02-0.1,-0.07,0], (np.pi,0,0))
                         ]),
        'box1':MarkerSet('box1',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05, 0.05,0.05), color=(0.8,0.3,0.3,1),
                         _list=[
                             ObjectMarker('box1', 111, 0.04, [0,0,0.025], (np.pi,0,0)),
                             ObjectMarker('box1', 112, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                             ObjectMarker('box1', 113, 0.04, [0,0,-0.025], (0,0,0)),
                             ObjectMarker('box1', 114, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                             ObjectMarker('box1', 115, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                             ObjectMarker('box1', 116, 0.04, [-0.025,0,0], (0,np.pi/2,0))
                         ]),
        'box2':MarkerSet('box2',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05, 0.05,0.05), color=(0.3,0.3,0.8,1),
                         _list=[
                             ObjectMarker('box2', 121, 0.04, [0,0,0.025], (np.pi,0,0)),
                             ObjectMarker('box2', 122, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                             ObjectMarker('box2', 123, 0.04, [0,0,-0.025], (0,0,0)),
                             ObjectMarker('box2', 124, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                             ObjectMarker('box2', 125, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                             ObjectMarker('box2', 126, 0.04, [-0.025,0,0], (0,np.pi/2,0))
                         ]),
        'box3':MarkerSet('box3',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05,0.05,0.05), color=(0.3,0.8,0.3,1),
                         soft=False, #K_col=100,
                         _list=[
                             ObjectMarker('box3', 131, 0.04, [0,0,0.025], (np.pi,0,0)),
                             ObjectMarker('box3', 132, 0.04, [0,0.025,0], (np.pi/2,0,0)),
                             ObjectMarker('box3', 133, 0.04, [0,0,-0.025], (0,0,0)),
                             ObjectMarker('box3', 134, 0.04, [0,-0.025,0], (-np.pi/2,0,0)),
                             ObjectMarker('box3', 135, 0.04, [0.025,0,0], (0,-np.pi/2,0)),
                             ObjectMarker('box3', 136, 0.04, [-0.025,0,0], (0,np.pi/2,0))
                         ]),
        'long_1':MarkerSet('long_1',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05,0.05,0.10), color=(0.8,0.8,0.8,1),
                         _list=[
                             ObjectMarker('long_1', 140, 0.04, [0,0,0.05], (np.pi,0,0)),

                             ObjectMarker('long_1', 141, 0.04, [0,0.025,-0.025], (np.pi/2,0,0)),
                             ObjectMarker('long_1', 142, 0.04, [0,0.025,+0.025], (np.pi/2,0,0)),

                             ObjectMarker('long_1', 143, 0.04, [0,0,-0.05], (0,0,0)),

                             ObjectMarker('long_1', 144, 0.04, [0,-0.025,+0.025], (-np.pi/2,0,0)),
                             ObjectMarker('long_1', 145, 0.04, [0,-0.025,-0.025], (-np.pi/2,0,0)),

                             ObjectMarker('long_1', 146, 0.04, [0.025,0,-0.025], (0,-np.pi/2,0)),
                             ObjectMarker('long_1', 147, 0.04, [0.025,0,+0.025], (0,-np.pi/2,0)),

                             ObjectMarker('long_1', 148, 0.04, [-0.025,0,+0.025], (0,np.pi/2,0)),
                             ObjectMarker('long_1', 149, 0.04, [-0.025,0,-0.025], (0,np.pi/2,0))
                         ]),
        'long_2':MarkerSet('long_2',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.05,0.05,0.10), color=(0.8,0.8,0.8,1),
                         _list=[
                             ObjectMarker('long_2', 150, 0.04, [0,0,0.05], (np.pi,0,0)),

                             ObjectMarker('long_2', 152, 0.04, [0,0.025,+0.025], (np.pi/2,0,0)),
                             ObjectMarker('long_2', 153, 0.04, [0,0.025,-0.025], (np.pi/2,0,0)),

                             ObjectMarker('long_2', 155, 0.04, [0,0,-0.05], (0,0,0)),

                             ObjectMarker('long_2', 157, 0.04, [0,-0.025, 0.025], (-np.pi/2,0,0)),
                             ObjectMarker('long_2', 158, 0.04, [0,-0.025,-0.025], (-np.pi/2,0,0)),

                             ObjectMarker('long_2', 156, 0.04, [0.025,0,-0.025], (0,-np.pi/2,0)),
                             ObjectMarker('long_2', 161, 0.04, [0.025,0, 0.025], (0,-np.pi/2,0)),

                             ObjectMarker('long_2', 162, 0.04, [-0.025,0, 0.025], (0,np.pi/2,0)),
                             ObjectMarker('long_2', 163, 0.04, [-0.025,0,-0.025], (0,np.pi/2,0))
                         ]),
        'goal':MarkerSet('goal',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('goal', 201, 0.1, [0,0,0.005], (np.pi,0,0))
                         ]),
        'target1':MarkerSet('target1',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('target1', 211, 0.1, [0,-0.11,0.005], (np.pi,0,0))
                         ]),
        'target2':MarkerSet('target2',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('target2', 212, 0.1, [0,-0.11,0.005], (np.pi,0,0))
                         ]),
        'target3':MarkerSet('target3',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('target3', 213, 0.1, [0,-0.11,0.005], (np.pi,0,0))
                         ]),
        'target4':MarkerSet('target4',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('target4', 214, 0.1, [0,-0.11,0.005], (np.pi,0,0))
                         ]),
        'target5':MarkerSet('target5',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('target5', 215, 0.1, [0,-0.11,0.005], (np.pi,0,0))
                         ]),
        'mark1':MarkerSet('mark1',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('mark1', 221, 0.18, [0,0,0], (0,0,0))
                         ]),
        'mark2':MarkerSet('mark2',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('mark2', 222, 0.18, [0,0,0], (0,0,0))
                         ]),
        'mark3':MarkerSet('mark3',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('mark3', 223, 0.18, [0,0,0], (0,0,0))
                         ]),
        'mark4':MarkerSet('mark4',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('mark4', 224, 0.18, [0,0,0], (0,0,0))
                         ]),
        'chair':MarkerSet('chair',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('chair', 225, 0.18, [0,0,0], (0,0,0))
                         ]),
        'table':MarkerSet('table',
                         dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.1), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('table', 226, 0.18, [0,0,0], (0,0,0))
                         ]),
        'test_target': MarkerSet('test_target',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.1, 0.1,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('test_target', 220, 0.06, [0.0, 0.0, 0.0], (0, 0, 0))
                         ])
    })
    return aruco_map
