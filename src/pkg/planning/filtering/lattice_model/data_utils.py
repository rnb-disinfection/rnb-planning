import numpy as np
import pickle
from pkg.utils.rotation_utils import *

GRASP_SHAPE = (20,20,20)
ARM_SHAPE = (20,20,20)
RH_MASK_SIZE = 512
RH_MASK_STEP = 64

def load_pickle_py3(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f, encoding='bytes')
        return data

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def div_r_gaussian(r_val):
    return gaussian(r_val, np.arange(0.1,1.2, 0.05),0.1)

def div_h_gaussian(h_val):
    return gaussian(h_val, np.arange(-0.5,1.1, 0.05),0.1)

def load_data(data_pair):
    grasp_data = load_pickle_py3(data_pair[0])
    arm_data = load_pickle_py3(data_pair[1])
    grasp_obj_idx = grasp_data[b'obj']
    grasp_tar_idx = grasp_data[b'tar']
    grasp_tool_idx = grasp_data[b'tool']
    arm_tar_idx = arm_data[b'tar']
    Tee = grasp_data[b'T_end_effector']
    Tej = grasp_data[b'T_end_joint']
    Tref_base = grasp_data[b'Tref_base']
    reach_lb = grasp_data[b'reach']
    retrieve_lb = grasp_data[b'retrieve']
    r, th, h = cart2cyl(*Tee[:3,3])
#     r_ej, th_ej, h_ej = cart2cyl(*Tej[:3,3])  # not exact value, no use
    r_mask = div_r_gaussian(r)
    h_mask = div_h_gaussian(h)
    rh_mask = np.concatenate([r_mask, h_mask])
#     rh_mask = np.array([r, h, r_ej, h_ej])
    # r_ej_list.append(r_ej)
    # h_ej_list.append(h_ej)
    # reach_lb_list.append(reach_lb)
#     Tref = SE3(Rot_axis(3, th), Tee[:3,3])
    grasp_tool_img = np.zeros(GRASP_SHAPE)
    grasp_tar_img = np.zeros(GRASP_SHAPE)
    grasp_obj_img = np.zeros(GRASP_SHAPE)
    grasp_tool_img[np.unravel_index(grasp_tool_idx, shape=GRASP_SHAPE)] = 1
    grasp_tar_img[np.unravel_index(grasp_tar_idx, shape=GRASP_SHAPE)] = 1
    grasp_obj_img[np.unravel_index(grasp_obj_idx, shape=GRASP_SHAPE)] = 1
    arm_img = np.zeros(ARM_SHAPE+(1,))
    arm_img[np.unravel_index(arm_tar_idx, shape=ARM_SHAPE)] = 1
    grasp_img = np.stack([grasp_tool_img, grasp_obj_img, grasp_tar_img], axis=-1)
#     grasp_img = np.stack([grasp_tool_img, np.logical_or(grasp_obj_img, grasp_tar_img)], axis=-1)
#     grasp_img = np.stack([grasp_tool_img, grasp_tar_img], axis=-1)
    label = np.array([reach_lb, retrieve_lb])
    return grasp_img, arm_img, rh_mask, label


def mix_data_pairs(data_pairs_filtered, data_pairs_failmore, N_max):
    data_pairs_mixed = []
    for data_pairs in data_pairs_filtered:
        grasp_img, arm_img, rh_mask, label = load_data(data_pairs)
        if all(label):
            data_pairs_mixed.append(data_pairs)
            N_succ = len(data_pairs_mixed)
            if N_succ >= N_max/2:
                break
    print("filtered success: {}".format(N_succ))

    if N_succ < N_max/2:
        for data_pairs in data_pairs_failmore:
            grasp_img, arm_img, rh_mask, label = load_data(data_pairs)
            if all(label):
                data_pairs_mixed.append(data_pairs)
            if len(data_pairs_mixed)>=N_max/2:
                break
    N_newsuc = len(data_pairs_mixed) - N_succ
    N_succ = N_succ + N_newsuc
    print("success from failmore: {}".format(N_newsuc))

    for data_pairs in data_pairs_failmore:
        grasp_img, arm_img, rh_mask, label = load_data(data_pairs)
        if not all(label):
            data_pairs_mixed.append(data_pairs)
        if len(data_pairs_mixed)>=N_succ*2:
            break
    N_train = len(data_pairs_mixed)
    N_fail = N_train - N_succ
    print("dataset fails: {}".format(N_fail))
    print("dataset all: {}".format(N_train))
    return data_pairs_mixed
