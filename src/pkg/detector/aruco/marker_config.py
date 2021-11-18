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

        # for left
        'test_bed_left': MarkerSet('test_bed_left',
                                   dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                                   color=(0.8, 0.0, 0.0, 1),
                                   _list=[
                                       ObjectMarker('tbed_1', 230, 0.15,
                                                    [1.14752287491, -0.353252079486, 0.143498200623],
                                                    (-1.63635897793, 0.0263834133234, 1.59629242535)),
                                       ObjectMarker('tbed_2', 231, 0.15,
                                                    [1.15836964434, 0.36816338243, 0.133507330722],
                                                    (-1.62492067564, 0.0144625603136, 1.58790850547)),
                                       ObjectMarker('tbed_3', 232, 0.15,
                                                    [0.827344633706, -0.438209209602, 0.117748211003],
                                                    (-1.52968333198, 0.00322486012472, 0.0110016360024)),
                                       ObjectMarker('tbed_4', 233, 0.15,
                                                    [0.0686143318984, -0.440206429266, 0.0689999489732],
                                                    (-1.53697588049, -0.0199944442108, 0.0203078797412))
                                   ]),

        # for right
        'test_bed_right': MarkerSet('test_bed_right',
                              dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                              color=(0.8, 0.0, 0.0, 1),
                              _list=[
                                  ObjectMarker('tbed_1', 230, 0.15,
                                               [-1.0076472631, -0.0248945626471, 0.0214445358094],
                                               (-1.63363185629, 0.038088904798, 1.63625426107)),
                                  ObjectMarker('tbed_2', 231, 0.15,
                                               [-1.11685669059, -0.221073625658, 0.692911918323],
                                               (-1.70924655752, 0.228443846637, 2.74892699487)),
                                  ObjectMarker('tbed_3', 232, 0.15,
                                               [0.669339788935, 0.446540918527, 0.11562540081],
                                               (-1.58679602169, 0.033245814478, -3.12124460471)),
                                  ObjectMarker('tbed_4', 233, 0.15,
                                               [-0.316543948633, 0.399756293516, 0.0693476148821],
                                               (-1.51714858142, 0.0658343116654, 3.0535149404))
                              ]),

        # for left
        'test_closet_left': MarkerSet('test_closet_left',
                                dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                                color=(0.8, 0.0, 0.0, 1),
                                _list=[
                                    ObjectMarker('tcloset_1', 234, 0.15,
                                                 [0.277299134702, -0.126165211823, 1.04983652331],
                                                 (-1.62275156438, 0.0127476963125, 1.63408457792)),
                                    ObjectMarker('tcloset_2', 235, 0.15,
                                                 [0.245661357796, -0.120902472068, 1.58935509853],
                                                 (-1.59369984754, 0.00345111009109, 1.62508033859))
                                ]),

        # for right
        'test_closet_right': MarkerSet('test_closet_right',
                                       dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                                       color=(0.8, 0.0, 0.0, 1),
                                       _list=[
                                           ObjectMarker('tcloset_1', 234, 0.15,
                                                        [0.256264538333, -0.249479025429, 0.975150451812],
                                                        (-1.57311534323, -0.0278033201102, 1.50598396777)),
                                           ObjectMarker('tcloset_2', 235, 0.15,
                                                        [0.147344485014, -0.391550155724, 1.54127589363],
                                                        (-1.54250356449, -0.0276251720469, 1.52635253862))
                                       ]),

        # 'test_closet_right': MarkerSet('test_closet_right',
        #                               dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
        #                               color=(0.8, 0.0, 0.0, 1),
        #                               _list=[
        #                                   ObjectMarker('tcloset_1', 234, 0.15,
        #                                                [0.242582267934, -0.264566958949, 1.02859615138],
        #                                                (-1.55813729407, 0.0223252509431, 1.54556304021)),
        #                                   ObjectMarker('tcloset_2', 235, 0.15,
        #                                                [0.131722082574, -0.384731807789, 1.58214117406],
        #                                                (-1.53921009276, 0.0160478989766, 1.55223540379))
        #                               ])

        'test_toilet': MarkerSet('test_toilet',
                                   dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.BOX, dims=(0.6, 0.4, 0.01),
                                   color=(0.8, 0.0, 0.0, 1),
                                   _list=[
                                       ObjectMarker('ttoilet_1', 230, 0.15,
                                                    [0.0211527236025, 0.064904134794, 0.0505565627744],
                                                    (0.00873870643953, 0.120807498244, -3.0863425541)),
                                       ObjectMarker('ttoilet_2', 231, 0.15,
                                                    [0.498186623241, 0.0847299565966, 0.0885935863922],
                                                    (0.0338789876876, 0.0902080886993, -3.1031310663)),
                                       ObjectMarker('ttoilet_3', 232, 0.15,
                                                    [0.105553045934, 0.61795361593, 0.485479704806],
                                                    (-0.323776064386, 0.0938069048613, -3.01927412346))
                                   ])

    })
    return aruco_map
