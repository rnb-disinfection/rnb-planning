import time
import numpy as np
from .rotation_utils import *
from .utils import *

MIN_RADI_DEFAULT = 0.3
STEP_SIZE_DEFAULT = 0.05

def SE2(R, P):
    T = np.identity(3)
    if R is not None:
        T[:2,:2] = R
    if P is not None:
        T[:2,2] = P
    return T

def get_nonholo_trajargs(x1, x2):
    T1 = SE2(Rot_axis(3, x1[2])[:2,:2], x1[:2])
    T2 = SE2(Rot_axis(3, x2[2])[:2,:2], x2[:2])
    T12 = np.matmul(np.linalg.inv(T1), T2)
    T21 = np.matmul(np.linalg.inv(T2), T1)
    x12 = T12[:2, 2]
    x21 = T21[:2, 2]
    v12 = T12[:2, 0]
    v21 = T21[:2, 0]

    y12 = -x12[1]/(v12[1]+1e-16*sign_positive_bias(v12[1]))*v12[0]+x12[0]
    y21 = -x21[1]/(v21[1]+1e-16*sign_positive_bias(v21[1]))*v21[0]+x21[0] # Backward
    theta = np.arctan2(v12[1], v12[0])
    
    y0min = sorted([y12, y21], key=abs)[0]
    R = abs(y0min/(np.tan(abs(theta)/2)+1e-20))
    
    return y12, y21, theta, R, T1, T2

def dist_nonholo(x1, x2, min_radi=MIN_RADI_DEFAULT):
    y12, y21, theta, R, T1, T2 = get_nonholo_trajargs(x1, x2)
    
    if abs(y12)>1e5 or abs(y21)>1e5 or y12*y21>0:
        L = 1e10
    else:
        if R<min_radi:
            L = 1e10
        else:
            dy0 = y12+y21 # y-intercept difference
            L = R*abs(theta) + abs(dy0)
    return L

def calc_move_x(T, dist, step_size):
    step_size = sign_positive_bias(dist)*step_size
    T_list = []
    for dy in np.arange(0,dist+step_size/2, step_size):
        T_list.append(np.matmul(T, SE2(None, [dy, 0])))
#     print(len(T_list))
    return T_list

def calc_rotate_x(T, theta, R, step_size, backward=False):
    if abs(R)<1e-6 or not(-1e5<R<1e5):
        return [T]
    step_theta = sign_positive_bias(theta)*step_size/abs(R)
    To = (np.matmul(T, SE2(None, [0, R*sign_positive_bias(theta)])))
    T_list = []
    try:
        steps = np.arange(0,theta+step_theta/2, step_theta)
    except:
        print("steps error: ", theta, step_theta, R)
    for theta_i in steps:
        T_list.append(matmul_series(To, 
                                    SE2(Rot_axis(3, theta_i)[:2,:2], None),
                                    SE2(None, [0, -sign_positive_bias(theta)*R]))
                     )
#     print(len(T_list))
    return T_list

def interpolate_nonholo(y12, y21, theta, R, T1, T2, min_radi=MIN_RADI_DEFAULT, step_size=STEP_SIZE_DEFAULT):
    if  abs(y12)>1e5 or abs(y21)>1e5 or y12*y21 > 0 or abs(R)<min_radi:
        return []
    T_list = []    
    if y12>0:
#         print("forward")
        if abs(y12)>abs(y21):
#             print("forward first - {}, {}".format(y12, y21))
            T_list += calc_move_x(T1, y12+y21, step_size)
#             print("rotate next - {}, {}".format(theta, R))
            T_list += calc_rotate_x(T_list[-1], theta, R, step_size)
        else:
#             print("rotate first - {}, {}".format(theta, R))
            T_list += calc_rotate_x(T1, theta, R, step_size)
#             print("forward next - {}, {}".format(y12, y21))
            T_list += calc_move_x(T_list[-1], abs(y12+y21), step_size)
    else: 
#         print("backward")
        if abs(y12)>abs(y21): 
#             print("backward first - {}, {}".format(y12, y21))
            T_list += calc_move_x(T1, (y12+y21), step_size)
#             print("rotate next - {}, {}".format(theta, R))
            T_list += calc_rotate_x(T_list[-1], theta, -R, step_size)
        else:
#             print("rotate first - {}, {}".format(theta, R))
            T_list += calc_rotate_x(T1, theta, -R, step_size)
#             print("backward next - {}, {}".format(y12, y21))
            T_list += calc_move_x(T_list[-1], -(y12+y21), step_size)
    T_list.append(T2)
    return T_list