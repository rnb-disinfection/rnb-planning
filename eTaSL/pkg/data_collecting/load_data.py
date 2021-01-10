from pkg.utils.utils_python3 import *
import random
import shutil
import os

CONVERTED_PATH_DEFAULT = "./data/converted"
SCENE_FILENAME_DEFAULT = "scene.pkl"
    
    
class DataLoader:
    def __init__(self, JOINT_NUM, CONVERTED_PATH=CONVERTED_PATH_DEFAULT, SCENE_FILENAME=SCENE_FILENAME_DEFAULT,
                 load_limits=True, get_difference=True):
        self.JOINT_NUM, self.CONVERTED_PATH, self.SCENE_FILENAME = JOINT_NUM, CONVERTED_PATH, SCENE_FILENAME
        self.get_difference = get_difference
        self.load_limits = load_limits
        self.N_vtx_box = 3*8
        self.N_mask_box = 1
        self.N_joint_box = self.JOINT_NUM
        self.N_label_box = self.N_vtx_box+self.N_mask_box+self.N_joint_box
        self.N_vtx_cyl = 3*2+1
        self.N_mask_cyl = 1
        self.N_joint_cyl = self.JOINT_NUM
        self.N_label_cyl = self.N_vtx_cyl+self.N_mask_cyl+self.N_joint_cyl
        self.N_vtx_init = 3*8
        self.N_mask_init = 1
        self.N_joint_init = self.JOINT_NUM
        self.N_label_init = self.N_vtx_init+self.N_mask_init+self.N_joint_init
        self.N_vtx_goal = 3*8
        self.N_mask_goal = 1
        self.N_joint_goal = self.JOINT_NUM
        self.N_label_goal = self.N_vtx_goal+self.N_mask_goal+self.N_joint_goal
        self.N_joint_label = 6*self.JOINT_NUM
        if load_limits:
            if self.get_difference:
                self.N_joint_limits = 2*self.JOINT_NUM
            else:
                self.N_joint_limits = 3*self.JOINT_NUM
        else:
            self.N_joint_limits = 0
        self.N_BEGIN_CYL = self.N_label_box
        self.N_BEGIN_INIT = self.N_BEGIN_CYL+self.N_label_cyl
        self.N_BEGIN_GOAL = self.N_BEGIN_INIT+self.N_label_init
        self.N_BEGIN_JOINT = self.N_BEGIN_GOAL+self.N_label_goal
        self.N_cell_label = self.N_label_box+self.N_label_cyl+self.N_label_init+self.N_label_goal \
                            + self.N_joint_label + self.N_joint_limits
    
    # trainset
    def get_dataset_args(self, TRAINSET_LIST, JOINT_NUM):
        SCENE_TUPLE_LIST = []
        for DATASET in TRAINSET_LIST:
            CURRENT_PATH = os.path.join(self.CONVERTED_PATH, DATASET)
            #Iterate world
            WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))
            for WORLD in WORLD_LIST:
                WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
                # Iterate scene
                SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
                for SCENE in SCENE_LIST:
                    SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
                    ACTION_LIST = sorted(filter(lambda x: x != self.SCENE_FILENAME, os.listdir(SCENE_PATH)))
                    for ACTION in ACTION_LIST:
                        SCENE_TUPLE_LIST.append((self.CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION, self.JOINT_NUM))
        return SCENE_TUPLE_LIST
    
    def separate_dat(self, scene_data):
        cbox = scene_data[:,:,:,:, :self.N_vtx_box]
        cbox_m = scene_data[:,:,:,:, self.N_vtx_box]
        cbox_j = scene_data[:,:,:,:, self.N_vtx_box+1:self.N_vtx_box+1+self.N_joint_box]
        ccyl = scene_data[:,:,:,:, self.N_BEGIN_CYL:self.N_BEGIN_CYL+self.N_vtx_cyl]
        ccyl_m = scene_data[:,:,:,:, self.N_BEGIN_CYL+self.N_vtx_cyl]
        ccyl_j = scene_data[:,:,:,:, self.N_BEGIN_CYL+self.N_vtx_cyl+1:self.N_BEGIN_CYL+self.N_vtx_cyl+1+self.N_joint_cyl]
        ibox = scene_data[:,:,:,:, self.N_BEGIN_INIT:self.N_BEGIN_INIT+self.N_vtx_box]
        ibox_m = scene_data[:,:,:,:, self.N_BEGIN_INIT+self.N_vtx_box]
        ibox_j = scene_data[:,:,:,:, self.N_BEGIN_INIT+self.N_vtx_box+1:self.N_BEGIN_INIT+self.N_vtx_box+1+self.N_joint_init]
        gbox = scene_data[:,:,:,:, self.N_BEGIN_GOAL:self.N_BEGIN_GOAL+self.N_vtx_box]
        gbox_m = scene_data[:,:,:,:, self.N_BEGIN_GOAL+self.N_vtx_box]
        gbox_j = scene_data[:,:,:,:, self.N_BEGIN_GOAL+self.N_vtx_box+1:self.N_BEGIN_GOAL+self.N_vtx_box+1+self.N_joint_goal]
        if self.load_limits and self.get_difference:
            jvals = scene_data[:,:,:,:,self.N_BEGIN_JOINT+self.N_joint_label: self.N_BEGIN_JOINT+self.N_joint_label+self.JOINT_NUM]
            joints = np.reshape(scene_data[:,:,:,:,self.N_BEGIN_JOINT: self.N_BEGIN_JOINT+self.N_joint_label], scene_data.shape[:4]+(-1,int(self.N_joint_label/self.JOINT_NUM)))
            joints = np.concatenate([joints,
                                     np.stack(
                                         [jvals - scene_data[:,:,:,:,-self.N_joint_limits:-self.N_joint_limits+self.JOINT_NUM],
                                                     scene_data[:,:,:,:,-self.N_joint_limits+self.JOINT_NUM:] - jvals], axis=-1)
                                    ], axis=-1)
        else:
            joints = scene_data[:,:,:,:,self.N_BEGIN_JOINT:]
            joints = np.reshape(joints, scene_data.shape[:4]+(-1,int((self.N_joint_label+self.N_joint_limits)/self.JOINT_NUM)))
        cbox = np.concatenate([cbox, 
                               (joints*np.expand_dims(cbox_j, axis=-1)).reshape(scene_data.shape[:4]+(-1,))], 
                              axis=-1)
        ccyl = np.concatenate([ccyl, 
                               (joints*np.expand_dims(ccyl_j, axis=-1)).reshape(scene_data.shape[:4]+(-1,))], 
                              axis=-1)
        ibox = np.concatenate([ibox, 
                               (joints*np.expand_dims(ibox_j, axis=-1)).reshape(scene_data.shape[:4]+(-1,))], 
                              axis=-1)
        gbox = np.concatenate([gbox, 
                               (joints*np.expand_dims(gbox_j, axis=-1)).reshape(scene_data.shape[:4]+(-1,))], 
                              axis=-1)
        return (cbox, np.expand_dims(cbox_m, axis=-1), 
                ccyl, np.expand_dims(ccyl_m, axis=-1), 
                ibox, np.expand_dims(ibox_m, axis=-1), 
                gbox, np.expand_dims(gbox_m, axis=-1))
            
