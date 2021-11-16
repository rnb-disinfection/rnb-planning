from .detector import *
from ...geometry.geometry import GEOTYPE

def get_aruco_map():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    #     params = aruco.DetectorParameters_create()

    aruco_map = ArucoMap(dictionary=dictionary, _dict={
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
        'test_table': MarkerSet('test_table',
                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4,0.01), color=(0.8,0.0,0.0,1),
                         _list=[
                             ObjectMarker('ttable_1', 230, 0.055, [0.0275, -0.0275, 0.0], (np.pi, 0, 0)),
                             ObjectMarker('ttable_2', 231, 0.055, [0.1215, -0.0275, 0.0], (np.pi, 0, 0)),
                             ObjectMarker('ttable_3', 232, 0.055, [0.0275, -0.1425, 0.0], (np.pi, 0, 0)),
                             ObjectMarker('ttable_4', 233, 0.055, [0.1215, -0.1425, 0.0], (np.pi, 0, 0))
                         ]),
        # # for left
        # 'test_bed': MarkerSet('test_bed',
        #                         dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
        #                         color=(0.8, 0.0, 0.0, 1),
        #                         _list=[
        #                             ObjectMarker('tbed_1', 230, 0.15,
        #                                          [1.18627376517, -0.342703558662, 0.120225254275],
        #                                          (1.52131995759, 0.0133979864499, 1.60435635356)),
        #                             ObjectMarker('tbed_2', 231, 0.15,
        #                                          [1.19599951174, 0.374033144349, 0.122950419945],
        #                                          (1.54346588349, -0.00874319802384, 1.58756294403)),
        #                             ObjectMarker('tbed_3', 232, 0.15,
        #                                          [0.889404125146, -0.43589665181, 0.108971108262],
        #                                          (1.55933691734, -0.0373457612565, -0.0184408198392)),
        #                             ObjectMarker('tbed_4', 233, 0.15,
        #                                          [0.140229213529, -0.431581490143, 0.0705030629548],
        #                                          (1.61742523905, -0.0140970518267, 0.0192163201867))
        #                         ]),

        # for left
        'test_bed': MarkerSet('test_bed',
                              dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                              color=(0.8, 0.0, 0.0, 1),
                              _list=[
                                  ObjectMarker('tbed_1', 230, 0.15,
                                               [1.2002175, -0.33125859, 0.17261603],
                                               (-1.65977093, 0.0459408, 1.59540118)),
                                  ObjectMarker('tbed_2', 231, 0.15,
                                               [1.20746026, 0.39556057, 0.14577929],
                                               (-1.64092644, 0.03097502, 1.58835796)),
                                  ObjectMarker('tbed_3', 232, 0.15,
                                               [0.89416823, -0.42463095, 0.15864651],
                                               (-1.53864032, 0.00202508, 0.00342758)),
                                  ObjectMarker('tbed_4', 233, 0.15,
                                               [0.103883083945, -0.40565275112, 0.0920507654886],
                                               (-1.55411966732, -0.0210382301981, 0.0141786053189))
                              ]),

        # # for right
        # 'test_bed': MarkerSet('test_bed',
        #                       dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
        #                       color=(0.8, 0.0, 0.0, 1),
        #                       _list=[
        #                           ObjectMarker('tbed_1', 230, 0.15,
        #                                        [-0.922045680246, -0.00717801432787, -0.0317057241136],
        #                                        (1.5139852756, -0.00210178048308, 1.66519479886)),
        #                           ObjectMarker('tbed_2', 231, 0.15,
        #                                        [-1.0871957852, -0.267970363843, 0.610030134852],
        #                                        (1.38293833269, 0.248477208201, 2.81820774128)),
        #                           ObjectMarker('tbed_3', 232, 0.15,
        #                                        [0.709012335191, 0.468341523168, 0.133590886149],
        #                                        (1.51925169422, 0.0657520360644, -3.1087491943)),
        #                           ObjectMarker('tbed_4', 233, 0.15,
        #                                        [-0.332366535353, 0.352724368267, 0.0154711434349],
        #                                        (1.06139188259, -0.510193426712, 1.47218931273))
        #                       ]),

        'test_closet': MarkerSet('test_closet',
                                dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                                color=(0.8, 0.0, 0.0, 1),
                                _list=[
                                    ObjectMarker('tcloset_1', 234, 0.15, [0.0275, -0.0275, 0.0], (np.pi, 0, 0)),
                                    ObjectMarker('tcloset_2', 235, 0.15, [0.1215, -0.0275, 0.0], (np.pi, 0, 0))
                                ])
    })
    return aruco_map
