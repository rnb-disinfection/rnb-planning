'''
Created on 2019. 3. 15.

@author: JSK
'''
from scipy.spatial.transform import Rotation
import numpy as np
from math import *

def rad2deg(rads):
    return rads/np.pi*180
        
def deg2rad(degs):
    return degs/180*np.pi

def Rot_axis( axis, q ):
    '''
    make rotation matrix along axis
    '''
    if axis==1:
        R = np.asarray([[1,0,0],
                        [0,cos(q),-sin(q)],
                        [0,sin(q),cos(q)]])
    if axis==2:
        R = np.asarray([[cos(q),0,sin(q)],
                        [0,1,0],
                        [-sin(q),0,cos(q)]])
    if axis==3:
        R = np.asarray([[cos(q),-sin(q),0],
                        [sin(q),cos(q),0],
                        [0,0,1]])
    return R

def Rot_axis_series(axis_list, rad_list):
    '''
    zyx rotation matrix - caution: axis order: z,y,x
    '''
    R = Rot_axis(axis_list[0], rad_list[0])
    for ax_i, rad_i in zip(axis_list[1:], rad_list[1:]):
        R = np.matmul(R, Rot_axis(ax_i,rad_i))
    return R

def Rot_zyx(zr,yr,xr):
    '''
    zyx rotation matrix - caution: axis order: z,y,x
    '''
    return Rot_axis_series([3,2,1], [zr,yr,xr])

def Rot_zxz(zr1,xr2,zr3):
    '''
    zxz rotation matrix - caution: axis order: z,x,z
    '''
    return Rot_axis_series([3,1,3], [zr1,xr2,zr3])

def Rot2zyx(R):
    '''
    rotation matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,0]**2 + R[1,0]**2)

    if sy > 0.000001:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.asarray([z,y,x])

def Rot2zxz(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,2]**2 + R[1,2]**2)

    if sy > 0.000001:
        z1 = atan2(R[0,2] , -R[1,2])
        x2 = atan2(sy,R[2,2])
        z3 = atan2(R[2,0], R[2,1])
    else:
        z1 = 0
        x2 = atan2(sy,R[2,2])
        z3 = atan2(-R[0,1], R[0,0])
    return np.asarray([z1,x2,z3])

def SE3(R,P):
    T = np.identity(4,dtype='float32')
    T[0:3,0:3]=R
    T[0:3,3]=P
    return T

def SE3_inv(T):
    R=T[0:3,0:3].transpose()
    P=-np.matmul(R,T[0:3,3])
    return (SE3(R,P))

def SE3_R(T):
    return T[0:3,0:3]

def SE3_P(T):
    return T[0:3,3]

def SE3_mul_vec3(T,v):
    r=np.matmul(SE3_R(T),v)
    return np.add(r,SE3_P(T))

def average_SE3(Ts):
    nT = Ts.shape[0]
    Rref = Ts[0,:3,:3]
    dRlie_list = []
    for i in range(nT):
        Ri = Ts[i,:3,:3]
        dRi = np.matmul(Rref.transpose(),Ri)
        dRlie = np.real(scipy.linalg.logm(dRi,disp=False)[0])
        dRlie_list += [dRlie]
    dRlie_list = np.array(dRlie_list)
    dRlie_m = np.mean(dRlie_list,axis=0)
    R_m = np.matmul(Rref,scipy.linalg.expm(dRlie_m))
    P_m = np.mean(Ts[:,:3,3],axis=0)
    T_m=SE3(R_m,P_m)
    return T_m

def align_z(Two):
    Rwo = Two[0:3,0:3]
    Zwo=np.matmul(Rwo,[[0],[0],[1]])
    azim=np.arctan2(Zwo[1],Zwo[0])-np.deg2rad(90)
    altit=np.arctan2(np.linalg.norm(Zwo[:2]),Zwo[2])
    Rwo_=np.matmul(Rot_zxz(azim,altit,-azim),Rwo)
    Two_out = Two.copy()
    Two_out[0:3,0:3] = Rwo_
    return Two_out

def fit_floor(Tcw, Tco, minz):
    Pco = Tco[0:3,3]
    Twc = np.linalg.inv(Tcw)
    Pco_wz = np.dot(Twc[2,0:3],Pco)
    if abs(Pco_wz)<0.00001:
        Pco_wz = 0.00001
    alpha = abs((-minz - Twc[2,3])/Pco_wz)
    Pco_ = Pco*alpha
    Tco_out = Tco.copy()
    Tco_out[0:3,3]=Pco_
    return Tco_out

def project_px(Tco, cam_K, points):
    vtx_cco = np.matmul(cam_K, np.matmul(Tco[:3, :3], points.transpose()) + Tco[:3, 3:4])
    vtx_px = (vtx_cco[:2, :] / vtx_cco[2:3, :])
    return vtx_px, vtx_cco[2, :]

def Rot_rpy(rpy):
    return np.transpose(Rot_axis_series([1,2,3],np.negative(rpy)))

def Rot2rpy(R):
    return np.asarray(list(reversed(Rot2zyx(R))))

def T2xyzrpy(T):
    return T[:3,3].tolist(), Rot2rpy(T[:3,:3]).tolist()

def T2xyzrvec(T):
    return T[:3,3].tolist(), Rotation.from_dcm(T[:3,:3]).as_rotvec().tolist()

def matmul_series(*Tlist):
    T = Tlist[0]
    for T_i in Tlist[1:]:
        T = np.matmul(T, T_i)
    return T

Tx180 = np.identity(4, 'float32')
Tx180[1,1]=-1
Tx180[2,2]=-1

Ty180 = np.identity(4, 'float32')
Ty180[0,0]=-1
Ty180[2,2]=-1