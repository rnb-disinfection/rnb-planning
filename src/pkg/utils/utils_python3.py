import os
import pickle
import dill

dill._dill._reverse_typemap['ObjectType'] = object
dill._dill._reverse_typemap[b'ListType'] = list

import time
import numpy as np
import collections


##
# @class    GlobalTimer
# @brief    A singleton timer to record timings anywhere in the code.
# @remark   Call GlobalTimer.instance() to get the singleton timer.
#           To see the recorded times, just print the timer: print(global_timer)
class GlobalTimer:
    def __init__(self, scale=1000, timeunit='ms'):
        self.reset(scale, timeunit)

    ##
    # @brief    reset the timer.
    # @param    scale       scale of the timer compared to a second. For ms timer, 1000
    # @param    timeunit    name of time unit for printing the log
    def reset(self, scale=1000, timeunit='ms'):
        self.scale = scale
        self.timeunit = timeunit
        self.name_list = []
        self.ts_dict = {}
        self.time_dict = collections.defaultdict(lambda: 0)
        self.min_time_dict = collections.defaultdict(lambda: 1e10)
        self.max_time_dict = collections.defaultdict(lambda: 0)
        self.count_dict = collections.defaultdict(lambda: 0)
        self.timelist_dict = collections.defaultdict(list)
        self.switch(True)

    ##
    # @brief    switch for recording time. switch-off to prevent time recording for optimal performance
    def switch(self, onoff):
        self.__on = onoff

    ##
    # @brief    mark starting point of time record
    # @param    name    name of the section to record time.
    def tic(self, name):
        if self.__on:
            if name not in self.name_list:
                self.name_list.append(name)
            self.ts_dict[name] = time.time()

    ##
    # @brief    record the time passed from last call of tic with same name
    # @param    name    name of the section to record time
    # @param    stack   to stack each time duration to timelist_dict, set this value to True
    def toc(self, name, stack=False):
        if self.__on:
            dt = (time.time() - self.ts_dict[name]) * self.scale
            self.time_dict[name] = self.time_dict[name] + dt
            self.min_time_dict[name] = min(self.min_time_dict[name], dt)
            self.max_time_dict[name] = max(self.max_time_dict[name], dt)
            self.count_dict[name] = self.count_dict[name] + 1
            if stack:
                self.timelist_dict[name].append(dt)
            return dt

    ##
    # @brief    record and start next timer in a line.
    def toctic(self, name_toc, name_tic, stack=False):
        dt = self.toc(name_toc, stack=stack)
        self.tic(name_tic)
        return dt

    ##
    # @brief you can just print the timer instance to see the record
    def __str__(self):
        strout = ""
        names = self.name_list
        for name in names:
            strout += "{name}: \t{tot_T} {timeunit}/{tot_C} = {per_T} {timeunit} ({minT}/{maxT})\n".format(
                name=name, tot_T=np.round(np.sum(self.time_dict[name])), tot_C=self.count_dict[name],
                per_T=np.round(np.sum(self.time_dict[name]) / self.count_dict[name], 3),
                timeunit=self.timeunit, minT=round(self.min_time_dict[name], 3), maxT=round(self.max_time_dict[name], 3)
            )
        return strout


import json

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def save_json(filename, data):
    with open(filename, "w") as json_file:
        json.dump(data, json_file, cls=NumpyEncoder,indent=2)
    
def load_json(filename):
    with open(filename, "r") as st_json:
        st_python = json.load(st_json)
    return st_python

def load_pickle(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f, encoding='bytes')
        return data


def load_scene_data(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION, joint_num, get_deviation=False):
    N_vtx_box = 3 * 8
    N_mask_box = 1
    N_joint_box = joint_num
    N_label_box = N_vtx_box + N_mask_box + N_joint_box
    N_vtx_cyl = 3 * 2 + 1
    N_mask_cyl = 1
    N_joint_cyl = joint_num
    N_label_cyl = N_vtx_cyl + N_mask_cyl + N_joint_cyl
    N_vtx_init = 3 * 8
    N_mask_init = 1
    N_joint_init = joint_num
    N_label_init = N_vtx_init + N_mask_init + N_joint_init
    N_vtx_goal = 3 * 8
    N_mask_goal = 1
    N_joint_goal = joint_num
    N_label_goal = N_vtx_goal + N_mask_goal + N_joint_goal
    N_joint_label = 6 * joint_num
    N_cell_label = N_label_box + N_label_cyl + N_label_init + N_label_goal + N_joint_label
    N_BEGIN_CYL = N_vtx_box + N_mask_box + N_joint_box
    N_BEGIN_INIT = N_BEGIN_CYL + N_vtx_cyl + N_mask_cyl + N_joint_cyl
    N_BEGIN_GOAL = N_BEGIN_INIT + N_vtx_init + N_mask_init + N_joint_init

    # print("load: {}".format((CONVERTED_PATH, DATASET, WORLD, SCENE)))
    scene_pickle = load_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, "scene.pkl"))
    scene_data = scene_pickle[b'scene_data']
    ctem_names = scene_pickle[b'ctem_names']
    ctem_cells = scene_pickle[b'ctem_cells']

    act_dat = load_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION))
    init_box_dat = act_dat[b'init_box_dat']
    goal_box_dat = act_dat[b'goal_box_dat']
    ctem_dat_list = act_dat[b'ctem_dat_list']
    skey = int(act_dat[b'skey'])
    success = act_dat[b'success']
    ### put init, goal item data
    cell, verts, chain = init_box_dat
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_INIT:N_BEGIN_INIT + N_vtx_init] = verts
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_INIT + N_vtx_init:N_BEGIN_INIT + N_vtx_init + N_mask_init] = 1
    scene_data[cell[0], cell[1], cell[2],
    N_BEGIN_INIT + N_vtx_init + N_mask_init:N_BEGIN_INIT + N_vtx_init + N_mask_init + N_joint_init] = chain
    cell_init = cell

    cell, verts, chain = goal_box_dat
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_GOAL:N_BEGIN_GOAL + N_vtx_goal] = verts
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_GOAL + N_vtx_goal:N_BEGIN_GOAL + N_vtx_goal + N_mask_goal] = 1
    scene_data[cell[0], cell[1], cell[2],
    N_BEGIN_GOAL + N_vtx_goal + N_mask_goal:N_BEGIN_GOAL + N_vtx_goal + N_mask_goal + N_joint_goal] = chain
    cell_goal = cell

    ### add/replace collilsion object
    for cname, ctype, cell, verts, chain in ctem_dat_list:
        if ctype == b'BOX':
            N_BEGIN_REP, N_vtx, N_mask, N_joint = 0, N_vtx_box, N_mask_box, N_joint_box
        elif ctype == b'CYLINDER':
            N_BEGIN_REP, N_vtx, N_mask, N_joint = N_BEGIN_CYL, N_vtx_cyl, N_mask_cyl, N_joint_cyl
        else:
            raise (RuntimeError("Non considered shape key"))
        scene_data[cell[0], cell[1], cell[2], N_BEGIN_REP:N_BEGIN_REP + N_vtx] = verts
        scene_data[cell[0], cell[1], cell[2], N_BEGIN_REP + N_vtx:N_BEGIN_REP + N_vtx + N_mask] = 1
        scene_data[cell[0], cell[1], cell[2],
        N_BEGIN_REP + N_vtx + N_mask:N_BEGIN_REP + N_vtx + N_mask + N_joint] = chain
    if get_deviation:
        return scene_data, success, skey, cell_init, cell_goal
    else:
        return scene_data, success, skey