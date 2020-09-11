from scipy.spatial.transform import Rotation
from .rotation_utils import *
from .utils import *
from .global_config import *
import autograd.numpy as np
from autograd.numpy import linalg

LAST_LINE = np.array([[0, 0, 0, 1]])
R_ZERO = np.identity(3)

from pymanopt.manifolds import Rotations
from pymanopt.manifolds import Euclidean
from pymanopt.manifolds import Product
from pymanopt import Problem
from pymanopt.solvers import SteepestDescent

CALIB_TIMEOUT = 5

def set_RP_calib(sample_list):
    global Rcam_list, Pcam_list, Rcal_list, Pcal_list, Rcaminv_list, Pcaminv_list, N_POS_CALIB
    Rcam_list, Pcam_list, Rcal_list, Pcal_list = [], [], [], []

    for sample in sample_list:
        xyz_cam, rvec_cam, xyz_cal, rvec_cal, _, _ = sample
        Pcam_list.append(xyz_cam[0])
        Rcam_list.append(Rotation.from_rotvec(rvec_cam[0]).as_dcm())
        Pcal_list.append(xyz_cal[0])
        Rcal_list.append(Rotation.from_rotvec(rvec_cal[0]).as_dcm())

    Rcam_list = np.array(Rcam_list)
    Pcam_list = np.expand_dims(Pcam_list, axis=-1)
    Rcal_list = np.array(Rcal_list)
    Pcal_list = np.expand_dims(Pcal_list, axis=-1)
    Rcaminv_list = np.array([Rcam.transpose() for Rcam in Rcam_list])
    Pcaminv_list = np.array([-np.matmul(Rcaminv, Pcam) for Rcaminv, Pcam in zip(Rcaminv_list, Pcam_list)])

    N_POS_CALIB = len(Pcaminv_list)


def T_err(T):
    Rbbbr = T[0]
    Pbbbr = np.expand_dims(T[1], axis=1)
    Rreeo = T[2]
    Preeo = np.expand_dims(T[3], axis=1)

    Rop_off_cam = matmul_md(Rcaminv_list, np.expand_dims(Rbbbr, axis=0))
    Pop_off_cam = matmul_md(Rcaminv_list, np.expand_dims(Pbbbr, axis=0)) + Pcaminv_list

    Ropo = matmul_md(Rop_off_cam, Rcal_list)
    Popo = matmul_md(Rop_off_cam, Pcal_list) + Pop_off_cam

    Ropo_off = matmul_md(Ropo, np.expand_dims(Rreeo, axis=0))
    Popo_off = matmul_md(Ropo, np.expand_dims(Preeo, axis=0)) + Popo

    d_Ropo_off = Ropo_off - R_ZERO
    err_R = np.sum(d_Ropo_off * d_Ropo_off)
    err_P = np.sum(Popo_off * Popo_off)

    err_tot = (err_R + err_P) / N_POS_CALIB
    return err_tot

def calibrate_offset():
    # (1) Instantiate a manifold
    manifold = Product((Rotations(3), Euclidean(3), Rotations(3), Euclidean(3)))

    # (2) Define the cost function (here using autograd.numpy)
    # def cost(T): return np.sum(T[0])+np.sum(T[1])

    problem = Problem(manifold=manifold, cost=T_err)

    # (3) Instantiate a Pymanopt solver
    solver = SteepestDescent(mingradnorm=1e-5, maxtime=CALIB_TIMEOUT, logverbosity=2)

    # let Pymanopt do the rest
    Xopt, optlog = solver.solve(problem)

    if 'max time' in optlog['stoppingreason']:
        raise (RuntimeError('Timeout: Not converged!'))
    print('Calibration error: ' + str(optlog['final_values']['f(x)']))
    if optlog['final_values']['f(x)'] > 5e-4:
        print('Error too big! : ' + str(optlog['final_values']['f(x)']))

    T_bbbr = SE3(Xopt[0], Xopt[1])
    T_reeo = SE3(Xopt[2], Xopt[3])
    print("T_bbbr: {}".format(T_bbbr.astype(np.float16)))
    print("T_reeo: {}".format(T_reeo.astype(np.float16)))
    return T_bbbr, T_reeo


def save_offset(rname, T_bbbr, T_reeo, Teo_ref):
    np.savetxt(OFFSET_DIR + "T_bbbr_{rname}.csv".format(rname=rname), T_bbbr, delimiter=",")
    np.savetxt(OFFSET_DIR + "T_reeo_{rname}.csv".format(rname=rname), T_reeo, delimiter=",")
    np.savetxt(OFFSET_DIR + "Teo_ref_{rname}.csv".format(rname=rname), Teo_ref, delimiter=",")

def load_offset(rname):
    T_bbbr = np.loadtxt(OFFSET_DIR + "T_bbbr_{rname}.csv".format(rname=rname), delimiter=",")
    T_reeo = np.loadtxt(OFFSET_DIR + "T_reeo_{rname}.csv".format(rname=rname), delimiter=",")
    Teo_ref = np.loadtxt(OFFSET_DIR + "Teo_ref_{rname}.csv".format(rname=rname), delimiter=",")
    return T_bbbr, T_reeo, Teo_ref

