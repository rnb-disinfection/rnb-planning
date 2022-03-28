import time
import numpy as np
from .rotation_utils import *
from .utils import *

# OLD
MIN_RADI_DEFAULT = 0.3
STEP_SIZE_DEFAULT = 0.05

def SE2(R, P):
    T = np.identity(3)
    if R is not None:
        T[:2,:2] = R
    if P is not None:
        T[:2,2] = P
    return T


##
# @brief calculate simple distance for nonholonomic planning
# @param X0   initial config = [X, Y, heading]. heading in radian
# @param Xn   final config = [X, Y, heading]. heading in radian
# @param rot_scale  scale of rotation when calculating distance
def calc_nonolho_dist(X0, Xn, rot_scale=0.5):
    dist = np.linalg.norm(np.subtract(Xn[:2], X0[:2]))
    dH = np.abs(calc_rotvec_vecs([np.cos(X0[2]), np.sin(X0[2])], [np.cos(Xn[2]), np.sin(Xn[2])]))
    return dist + dH*rot_scale

##
# @brief interpolate with nonholonomic constraint by least-norm solution
# @param X0   initial config = [X, Y, heading]. heading in radian
# @param Xn   final config = [X, Y, heading]. heading in radian
# @param N    number of interpolation steps
# @param ref_step   reference step size
# @param rot_scale  scale of rotation when calculating distance
def interpolate_nonholo_leastnorm(X0, Xn, ref_step=0.1, N=None, rot_scale=0.5, min_radi=MIN_RADI_DEFAULT):
    assert N is not None or ref_step is not None, "[Error] Either N or ref_step should be passed"
    
    X0_ = np.copy(X0)

    if N is None:
        N = max(int(calc_nonolho_dist(X0, Xn, rot_scale) / ref_step), 1)

    X0 = np.concatenate([X0[:2], [np.cos(X0[2]), np.sin(X0[2])]])
    Xn = np.concatenate([Xn[:2], [np.cos(Xn[2]), np.sin(Xn[2])]])
                
    if ref_step is None:
        ref_step = np.linalg.norm(np.subtract(Xn, X0)[:2])/N
        
    depart_dir = sign_positive_bias(np.dot(X0[2:], np.subtract(Xn, X0)[:2]))
    
    X0[2:] *= depart_dir
    Xn[2:] *= depart_dir
        
    A = np.array(
        [[1,0,ref_step,0],
         [0,1,0,ref_step],
         [0,0,1,0],
         [0,0,0,1]])
    
    A_i = np.identity(4)
    A_accum = []
    for i_rev in range(N):
        A_accum.append(A_i[:,2:])
        A_i = np.matmul(A_i, A)
    A_accum = np.concatenate(A_accum, axis=1)
    dH_arr = np.matmul(np.linalg.pinv(A_accum), Xn - np.matmul(A_i, X0))
    
    dH_list = []
    for i in range(N):
        dH_list.append(dH_arr[i*2:(i+1)*2])
    dH_list = list(reversed(dH_list))    
    
    X_list = [X0_]
    Xpre_ = np.copy(X0_)
    X = np.copy(X0)
    for dH in dH_list:
        X = np.matmul(A, X)
        X_ = np.concatenate([X[:2], [np.arctan2(depart_dir*X[3], depart_dir*X[2])]])
        dX_ = (X_ - Xpre_)
        dX_[2] = (dX_[2]+np.pi)%(2*np.pi)-np.pi
        radi = np.linalg.norm(dX_[:2]) / (np.abs(dX_[2])+1e-16)
        if radi < min_radi:
            # print("fail - radi: {} / {}".format(np.round(radi, 3), np.round(min_radi, 3)))
            return None
        Xpre_ = X_
        X_list.append(X_)
        X[2:] += dH
    X_list = np.array(X_list)
    
    return X_list

### Old version
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