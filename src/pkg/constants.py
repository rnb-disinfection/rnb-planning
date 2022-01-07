import numpy as np

DIR_VEC_DICT = {"top": [0,0,1],
                "bottom": [0,0,-1],
                "right": [1,0,0],
                "left": [-1,0,0],
                "front": [0,-1,0],
                "back": [0,1,0]}

DIR_RPY_DICT = {"top": [np.pi,0,0],
                "bottom": [0,0,0],
                "right": [0,-np.pi/2,0],
                "left": [0,np.pi/2,0],
                "front": [-np.pi/2,0,0],
                "back": [np.pi/2,0,0]}

OPPOSITE_DICT={
    "top": "bottom",
    "bottom": "top",
    "right": "left",
    "left": "right",
    "front": "back",
    "back": "front"
}

