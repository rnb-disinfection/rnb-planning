DIR_VEC_DICT = {"top": [0,0,1],
                "bottom": [0,0,-1],
                "right": [1,0,0],
                "left": [-1,0,0],
                "front": [0,-1,0],
                "back": [0,1,0]}


from cv2.aruco import DetectorParameters_create
aruco_param = DetectorParameters_create()
aruco_param.adaptiveThreshConstant = 7.0
aruco_param.adaptiveThreshWinSizeMax = 23
aruco_param.adaptiveThreshWinSizeMin = 3
aruco_param.adaptiveThreshWinSizeStep = 10
aruco_param.cornerRefinementMaxIterations = 30
aruco_param.cornerRefinementMinAccuracy = 0.1
aruco_param.cornerRefinementWinSize = 5
aruco_param.doCornerRefinement = True
aruco_param.errorCorrectionRate = 0.6
aruco_param.markerBorderBits = 1
aruco_param.maxErroneousBitsInBorderRate = 0.35
aruco_param.maxMarkerPerimeterRate = 4.0
aruco_param.minCornerDistanceRate = 0.01
aruco_param.minDistanceToBorder = 3
aruco_param.minMarkerDistanceRate = 0.01
aruco_param.minMarkerPerimeterRate = 0.03
aruco_param.minOtsuStdDev = 5.0
aruco_param.perspectiveRemoveIgnoredMarginPerCell = 0.13
aruco_param.perspectiveRemovePixelPerCell = 4
aruco_param.polygonalApproxAccuracyRate = 0.01
