#!/usr/bin/env python
# coding: utf-8

# In[1]:


from __future__ import print_function
from IPython.core.display import display, HTML

display(HTML("<style>.container { width:90% !important; } </style>"))
import matplotlib.pyplot as plt
from pkg.marker_config import *
from pkg.constraint_graph import *
from pkg.constraint.constraint_action import *
from pkg.constraint.constraint_object import *
from pkg.constants import *
from pkg.utils.plot_utils import *
from pkg.utils.utils import *
from pkg.environment_builder import *
from pkg.ui.ui_broker import *
from pkg.controller.combined_robot import *
from pkg.controller.combined_robot import CombinedRobot, XYZ_RPY_ROBOTS_DEFAULT
from pkg.data_collecting.sampling import *

VISUALIZE = False
graph = None
SAMPLED_DATA = defaultdict(dict)
UPDATE_DAT = True

GLOBAL_FILENAME = "global.json"
WORLD_FILENAME = "world.json"
SCENE_FILENAME = "scene.json"
DATA_PATH = "./data"
CONVERTED_PATH = "./data/converted"
try_mkdir(CONVERTED_PATH)

gtimer = GlobalTimer.instance()
elog = Logger()
crob = CombinedRobot(connection_list=(False, False))

CHECK_DICT = {}

# ## Load Global params
DATASET_LIST = sorted(filter(lambda x: x not in ["backup", "converted"], os.listdir(DATA_PATH)))


# ## dataset

# In[2]:


DATASET = DATASET_LIST[0]
CHECK_DICT[DATASET] = {}
CURRENT_PATH = os.path.join(DATA_PATH, DATASET)
try_mkdir(os.path.join(CONVERTED_PATH, DATASET))


## Load global params
GLOBAL_PARAMS = load_json(os.path.join(CURRENT_PATH, GLOBAL_FILENAME))
WDH = GLOBAL_PARAMS["WDH"]
L_CELL = GLOBAL_PARAMS["L_CELL"]
RATIO_DIMS = GLOBAL_PARAMS["RATIO_DIMS"]
REACH_OFFSET_DICT = GLOBAL_PARAMS["REACH_OFFSET_DICT"]
GRIPPER_REFS = GLOBAL_PARAMS["GRIPPER_REFS"]
BASE_LINK = GLOBAL_PARAMS["BASE_LINK"]
S_F_RATIO = GLOBAL_PARAMS["S_F_RATIO"]
TIMEOUT = GLOBAL_PARAMS["TIMEOUT"]

CENTER = tuple(np.divide(WDH, 2, dtype=np.float))
Ws, Ds, Hs = WDH
Nw, Nd, Nh = Nwdh = int(Ws / L_CELL), int(Ds / L_CELL), int(Hs / L_CELL)
L_MAX = L_CELL * RATIO_DIMS
print("scene size: {} ({},{},{})".format(Nw * Nd * Nh, Nw, Nd, Nh))


# ## world

# In[ ]:


#Iterate world
WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))

for WORLD in WORLD_LIST:
    CHECK_DICT[DATASET][WORLD] = {}
    WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
    try_mkdir(os.path.join(CONVERTED_PATH, DATASET, WORLD))
    SAMPLED_DATA["WORLD"] = load_json(os.path.join(WORLD_PATH, WORLD_FILENAME))
    Trbt_dict = SAMPLED_DATA["WORLD"]["Trbt_dict"]
    reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}

    cam = None
    # set urdf
    xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, Trbt_dict,
                                                                       crob.custom_limits, start_rviz=VISUALIZE)
    ghnd = GeometryHandle(urdf_content)
    time.sleep(2)

    # set graph
    graph = ConstraintGraph(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                            urdf_content=urdf_content, combined_robot=crob)
    graph.set_camera(cam)
    graph.set_cam_robot_collision(_add_cam_poles=False, color=(1, 1, 0, 0.3))
    if VISUALIZE: graph.set_rviz()

    # start UI
    ui_broker = UIBroker.instance()
    ui_broker.initialize(graph)
    ui_broker.start_server()

    # set rviz
    if VISUALIZE: graph.set_rviz(crob.home_pose)
    ui_broker.set_tables()

    for gripper in GRIPPER_REFS.values():
        graph.register_binder(name=gripper['bname'], _type=FramedTool, point=gripper['tcp_ref'], rpy=[0, 0, 0],
                              link_name=gripper['link_name'])
    graph.register_binder(name='base', _type=PlaceFrame, point=[0, 0, 0], rpy=[0, 0, 0], link_name=BASE_LINK)
    vtem = graph.ghnd.create_safe(name="virtual", gtype=GEOTYPE.SPHERE, link_name=BASE_LINK,
                                  dims=(0, 0, 0), center=(0, 0, 0), rpy=(0, 0, 0), collision=False, display=False
                                  )
    graph.add_object("virtual",
                     SingleHandleObject(_object=vtem,
                                        action_point=FramePoint(name="point", _object=vtem, point=(0, 0, 0),
                                                                rpy=(0, 0, 0), name_full=None)),
                     binding=("point", "base"))

    obj_list = []
    col_obj_list = []

    # Iterate scene
    SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
    for SCENE in SCENE_LIST:
        try_mkdir(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE))
        CHECK_DICT[DATASET][WORLD][SCENE] = {}
        SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
        SAMPLED_DATA["SCENE"] = load_json(os.path.join(SCENE_PATH, SCENE_FILENAME))
        Q_s = np.array(SAMPLED_DATA["SCENE"]["Q_s"])
        Q_s, links, link_verts, link_ctems, link_rads = sample_joint(graph, Q_s_loaded=Q_s)
        Q_s_dict = SAMPLED_DATA["SCENE"]["Q_s_dict"]
        obj_dat = SAMPLED_DATA["SCENE"]["obj_dat"]

        if VISUALIZE:
            graph.show_pose(Q_s)
            time.sleep(1)
            graph.show_pose(Q_s)
            time.sleep(1)
            graph.show_pose(Q_s)
        for obj in obj_list: graph.remove_geometry(obj)
        for odat in obj_dat:
            nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"],                                                     odat["color"], odat["center"], odat["rpy"]
            obj = graph.ghnd.create_safe(
                name="{}_{}_{}_{}".format(gtype.name, *nbox), link_name=BASE_LINK, gtype=gtype,
                center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
            obj_list.append(obj)
            graph.add_marker(obj, vis=VISUALIZE)

        for obj in col_obj_list: graph.remove_geometry(obj)

        if VISUALIZE: graph.set_rviz()
        dcol = DataCollector(graph, GRIPPER_REFS, S_F_RATIO=S_F_RATIO)
        if VISUALIZE: graph.set_rviz()

        # planners
        mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names, urdf_path=graph.urdf_path,
                              urdf_content=graph.urdf_content,
                              robot_names=graph.combined_robot.robot_names,
                              binder_links=[v.object.link_name for v in graph.binder_dict.values()],
                              ghnd=graph.ghnd)
        dual_mplan_dict = get_dual_planner_dict(GRIPPER_REFS, graph.ghnd, graph.urdf_content, graph.urdf_path,
                                                graph.link_names, graph.combined_robot.robot_names)

        # ## initialize scene params
        link_names = graph.link_names
        joint_names = crob.joint_names
        joint_num = crob.joint_num
        IGNORE_CTEMS = ["panda1_hand_Cylinder_1", 'panda1_hand_Cylinder_2']
        joint_index_dict = {joint.name:None for joint in urdf_content.joints}
        joint_index_dict.update({jname:idx for idx, jname in zip(range(joint_num), joint_names)})
        centers = get_centers(Nwdh, L_CELL)
        merge_pairs = get_merge_pairs(ghnd, BASE_LINK)
        merge_paired_ctems(graph=graph, merge_pairs=merge_pairs, VISUALIZE=VISUALIZE)
        for cname in IGNORE_CTEMS:
            graph.remove_geometry(graph.ghnd.NAME_DICT[cname])

        N_vtx_box = 3*8
        N_mask_box = 1
        N_joint_box = joint_num
        N_label_box = N_vtx_box+N_mask_box+N_joint_box

        N_vtx_cyl = 3*2+1
        N_mask_cyl = 1
        N_joint_cyl = joint_num
        N_label_cyl = N_vtx_cyl+N_mask_cyl+N_joint_cyl

        N_vtx_init = 3*8
        N_mask_init = 1
        N_joint_init = joint_num
        N_label_init = N_vtx_init+N_mask_init+N_joint_init

        N_vtx_goal = 3*8
        N_mask_goal = 1
        N_joint_goal = joint_num
        N_label_goal = N_vtx_goal+N_mask_goal+N_joint_goal

        N_joint_label = 6*joint_num

        N_cell_label = N_label_box+N_label_cyl+N_label_init+N_label_goal + N_joint_label

        gtimer.reset()
        gtimer.tic("test_links")
        Tlink_dict = {}
        chain_dict = {}
        Tj_arr = np.zeros((joint_num, 4, 4))
        Tj_inv_arr = np.zeros((joint_num, 4, 4))
        joint_axis_arr = np.zeros((joint_num,3))

        gtimer.tic("T_chain")
        Tlink_dict = build_T_chain(link_names, Q_s_dict, urdf_content)
        gtimer.toc("T_chain")

        for lname in link_names:
            gtimer.tic("get_chain")
            jnames = filter(lambda x: x in joint_names, 
                            urdf_content.get_chain(root=BASE_LINK, tip=lname, joints=True, links=False))
            gtimer.toc("get_chain")
            chain_dict[lname] = [1 if pj in jnames else 0 for pj in joint_names]
        gtimer.toc("test_links")

        gtimer.tic("test_joints")
        for i_j, jname in zip(range(joint_num), joint_names):
            joint = urdf_content.joint_map[jname]
            lname = joint.child
            Tj_arr[i_j,:,:] = Tlink_dict[lname]
            Tj_inv_arr[i_j,:,:] = SE3_inv(Tlink_dict[lname])
            joint_axis_arr[i_j] = joint.axis
        gtimer.toc("test_joints")
        gtimer.tic("calc_cell")
        __p = centers.reshape(Nwdh+(1,3)) - Tj_arr[:,:3,3].reshape((1,1,1,joint_num,3))
        __w = np.sum(Tj_arr[:,:3,:3] * joint_axis_arr.reshape(joint_num, 1, 3), axis=-1).reshape((1,1,1,joint_num,3))
        __w = np.tile(__w, Nwdh+(1,1))
        __v = np.cross(__w, __p)
        xi = np.concatenate([__w, __v],axis=-1)
        gtimer.toc("calc_cell")
        gtimer.tic("calc_ctems")
        ctem_names, ctem_TFs, ctem_dims, ctem_cells, ctem_links, ctem_joints, ctem_types =             defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), dict()
        for ctem in ghnd:
            if not ctem.collision:
                continue 
            key = gtype_to_otype(ctem.gtype).name
            ctem_names[key].append(ctem.name)
            Ttem = np.matmul(Tlink_dict[ctem.link_name], ctem.Toff)
            ctem_TFs[key].append(Ttem)
            ctem_dims[key].append(ctem.dims)
            ctem_cells[key].append(get_cell(Ttem[:3,3], L_CELL, Nwdh))
            ctem_links[key].append(ctem.link_name)
            ctem_joints[key].append(chain_dict[ctem.link_name])
            ctem_types[key] = ctem.gtype


        verts_dict = {}
        for key_cur in ctem_cells:
            ctem_TFs[key_cur] = np.array(ctem_TFs[key_cur])
            cell_array = np.array(ctem_cells[key_cur])
            all_rearranged = False
            while not all_rearranged:
                all_rearranged = True
                for cell in cell_array:
                    idxset = np.where(np.all(cell_array==cell, axis=-1))[0]
                    if len(idxset)>1:
                        all_rearranged = False
                        cell_array_bak = cell_array.copy()
                        cell_array = rearrange_cell_array(cell_array, idxset, L_CELL, Nwdh, ctem_TFs[key_cur], centers)
                        break
            ctem_cells[key_cur] = cell_array
            gtype = ctem_types[key_cur]
            TFs = ctem_TFs[key_cur]
            dims = np.array(ctem_dims[key_cur])
            default_vert = DEFAULT_VERT_DICT[gtype]
            verts_dim = default_vert.reshape((1,-1,3))*dims.reshape((-1,1,3))
            cell_array = ctem_cells[key_cur]
            verts = np.zeros_like(verts_dim)
            for iv in range(verts.shape[1]):
                verts[:,iv,:] = matmul_md(TFs[:,:3,:3], verts_dim[:,iv,:,np.newaxis])[:,:,0]+TFs[:,:3,3]
            verts_loc = (verts - (cell_array[:,np.newaxis,:]*L_CELL+L_CELL/2))
            verts_loc = verts_loc.reshape((verts_loc.shape[0], -1))
            if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
                verts_loc = np.concatenate([verts_loc, dims[:,0:1]], axis=-1)
            verts_dict[key_cur] = verts_loc
        gtimer.toc("calc_ctems")
                
        ### initialize data volume
        scene_data = np.zeros(Nwdh+(N_cell_label,))
        ### put cell joint data
        scene_data[:,:,:,-N_joint_label:] = xi.reshape(Nwdh+(N_joint_label,))
        ### put cell item data
        for verts, cell, chain in zip(verts_dict["BOX"], ctem_cells["BOX"], ctem_joints["BOX"]):
            scene_data[cell[0],cell[1],cell[2],:N_vtx_box] = verts
            scene_data[cell[0],cell[1],cell[2],N_vtx_box:N_vtx_box+N_mask_box] = 1
            scene_data[cell[0],cell[1],cell[2],N_vtx_box+N_mask_box:N_vtx_box+N_mask_box+N_joint_box] = chain

        N_BEGIN_CYL = N_vtx_box+N_mask_box+N_joint_box
        for verts, cell, chain in zip(verts_dict["CYLINDER"], ctem_cells["CYLINDER"], ctem_joints["CYLINDER"]):
            scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL:N_BEGIN_CYL+N_vtx_cyl] = verts
            scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL+N_vtx_cyl:N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl] = 1
            scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl:N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl+N_joint_cyl] = chain
        save_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, "scene.pkl"), {"scene_data": scene_data, "ctem_names": ctem_names, "ctem_cells":ctem_cells})

        # Iterate actions
        ACTION_LIST = sorted(filter(lambda x: x != SCENE_FILENAME, os.listdir(SCENE_PATH)))
        for ACTION in ACTION_LIST:
            print("[BEGIN] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
            CHECK_DICT[DATASET][WORLD][SCENE][ACTION] = {}
            snode_dict_bak = {int(k): v for k, v in load_json(os.path.join(SCENE_PATH, ACTION)).items()}
            dcol.snode_dict = dcol.manager.dict()
            for k, v in snode_dict_bak.items():
                dcol.snode_dict[k] = deepcopy(v)
            snode_keys = sorted(snode_dict_bak.keys())

            if snode_keys:
                sk0 = snode_keys[0]
                rname, tar, inhand_coll, obj_coll =                     snode_dict_bak[sk0]["rname1"], snode_dict_bak[sk0]["rname2"],                     snode_dict_bak[sk0]["obj1"]["collision"], snode_dict_bak[sk0]["obj2"]["collision"]
            else:
                print("[END] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
                print("No Item")


            if rname and tar:  # handover case
                action_type = "HANDOVER"
            elif inhand_coll:  # place case
                action_type = "PLACE"
            elif obj_coll:  # pick case
                action_type = "PICK"
            else:
                raise (RuntimeError("non-implemented case"))

            # dcol, GRIPPER_REFS, Q_s, dual_mplan_dict, mplan, ID=None, UPDATE_DAT=True, VISUALIZE=False, timeout=1
            ID=None

            graph = dcol.graph
            elog = Logger()
            acquired = False
            action_data_list = []
            for skey in sorted(dcol.snode_dict.keys()):
                try:
                    dcol.dict_lock.acquire()
                    acquired = True
                    snode = dcol.snode_dict[skey]
                    dcol.dict_lock.release()
                    acquired = False
                    rname, inhand, obj, tar, dims_bak, color_bak, succ, _ = load_manipulation_from_dict(snode,
                                                                                                        graph.ghnd)
                except Exception as e:
                    if acquired:
                        dcol.dict_lock.release()
                    if not elog.log(str(e)):
                        break
                    continue
                try:
                    init_box_dat=get_cell_data(inhand, L_CELL, Nwdh, Tlink_dict, chain_dict, gtype=GEOTYPE.BOX)
                    goal_box_dat=get_cell_data(obj, L_CELL, Nwdh, Tlink_dict, chain_dict, gtype=GEOTYPE.BOX)

                    if rname and tar:  # handover case
                        remove_map = [[], [0, 1]]
                        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict)) for ctem in [inhand, obj] if ctem.collision]
                        recover_tems = []
                        action_type = "HANDOVER"
                    elif inhand.collision:  # place case
                        remove_map = [[], [0, 1]]
                        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict)) for ctem in [inhand, obj] if ctem.collision]
                        recover_tems = []
                        action_type = "PLACE"
                    elif obj.collision:  # pick case
                        remove_map = [[1], [0]]
                        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict, 
                                                                cell=ctem_cells[gtype_to_otype(ctem.gtype).name][ctem_names[gtype_to_otype(ctem.gtype).name].index(ctem.name)])) for ctem in [inhand, obj] if ctem.collision]
                        recover_tems = [obj]
                        action_type = "PICK"
                    else:
                        remove_map = [[], [0,1]]
                        col_tem_dats = []
                        recover_tems = []
                        action_type = "None"
                        raise (RuntimeError("non-implemented case"))
                    ### put init, goal item data 
                    N_BEGIN_INIT = N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl+N_joint_cyl
                    cell, verts, chain = init_box_dat
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_INIT:N_BEGIN_INIT+N_vtx_init] = verts
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_INIT+N_vtx_init:N_BEGIN_INIT+N_vtx_init+N_mask_init] = 1
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_INIT+N_vtx_init+N_mask_init:N_BEGIN_INIT+N_vtx_init+N_mask_init+N_joint_init] = chain

                    N_BEGIN_GOAL = N_BEGIN_INIT+N_vtx_init+N_mask_init+N_joint_init
                    cell, verts, chain = goal_box_dat
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_GOAL:N_BEGIN_GOAL+N_vtx_goal] = verts
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_GOAL+N_vtx_goal:N_BEGIN_GOAL+N_vtx_goal+N_mask_goal] = 1
                    scene_data[cell[0],cell[1],cell[2],N_BEGIN_GOAL+N_vtx_goal+N_mask_goal:N_BEGIN_GOAL+N_vtx_goal+N_mask_goal+N_joint_goal] = chain

                    ### add/replace collilsion object
                    ctem_dat_list=[]
                    for cname, ctype, cdat in col_tem_dats:
                        cell, verts, chain = cdat
                        if ctype == "BOX":
                            N_BEGIN_REP, N_vtx, N_mask, N_joint = 0, N_vtx_box, N_mask_box, N_joint_box
                        elif ctype == "CYLINDER":
                            N_BEGIN_REP, N_vtx, N_mask, N_joint = N_BEGIN_CYL, N_vtx_cyl, N_mask_cyl, N_joint_cyl
                        else:
                            raise(RuntimeError("Non considered shape key"))    
                        if scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask-1]:
                            if cname not in ctem_names[ctype]:
                                near_range = np.clip(
                                    ((cell[0]-1,cell[0]+1),(cell[1]-1,cell[1]+1),(cell[2]-1,cell[2]+1)), 
                                    [[0,0]]*3, np.transpose([Nwdh]*2)-1)
                                marks_near = scene_data[near_range[0][0]:near_range[0][1]+1,near_range[1][0]:near_range[1][1]+1,near_range[2][0]:near_range[2][1]+1,N_BEGIN_REP+N_vtx+N_mask-1]
                                marks_flt = marks_near.flatten()
                                idx_free = np.where(np.logical_not(marks_flt))[0]
                                near_cell_min = np.subtract(near_range,np.array(cell)[:,np.newaxis])[:,0]
                                cells_near = get_centers(marks_near.shape, 1)-0.5+near_cell_min
                                centers_cells = cells_near*L_CELL
                                centers_flt = centers_cells.reshape((-1,3))[idx_free]

                                if ctype == "CYLINDER":
                                    verts_res = verts[:-1].reshape((-1,3))
                                else:
                                    verts_res = verts.reshape((-1,3))
                                center_loc = np.mean(verts_res, axis=0)
                                if len(idx_free)==0:
                                    raise(RuntimeError("No free nearby cell"))
                                idx_cell = idx_free[np.argmin(np.linalg.norm(centers_flt-center_loc, axis=-1))]
                                cell_new = cells_near.reshape((-1,3))[idx_cell] + cell
                                diff = np.subtract(cell, cell_new)*L_CELL
                                if ctype == "CYLINDER":
                                    verts[:-1] = (verts_res + diff).flatten()
                                else:
                                    verts = (verts_res + diff).flatten()
                                cell_bak = deepcopy(cell)
                                cell = tuple(cell_new.astype(np.int))
                                if scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask-1]:
                                    print("wrong")
                                    raise(RuntimeError("collision item cell occupied"))        
                        ctem_dat_list.append((cname, ctype, cell, verts, chain))
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP:N_BEGIN_REP+N_vtx] = verts
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx:N_BEGIN_REP+N_vtx+N_mask] = 1
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask:N_BEGIN_REP+N_vtx+N_mask+N_joint] = chain
                    action_data_list.append({"init_box_dat":init_box_dat, "goal_box_dat":goal_box_dat, "ctem_dat_list":ctem_dat_list, "success":succ})
                except Exception as e:
                    if acquired:
                        dcol.dict_lock.release()
                    if not elog.log(str(e)):
                        break
                finally:
                    remove1 = [[inhand, obj][iii] for iii in remove_map[0]]
                    remove2 = [[inhand, obj][iii] for iii in remove_map[1]]
                    reset_rendering(graph, action_type, remove1, remove2, dims_bak, color_bak, sleep=True,
                                    vis=VISUALIZE)

                    ### recover collilsion object
                    recover_dat_list = []
                    for rtem in recover_tems:
                        rtype = gtype_to_otype(rtem.gtype).name
                        if rtype == "BOX":
                            N_BEGIN_REP, N_vtx, N_mask, N_joint = 0, N_vtx_box, N_mask_box, N_joint_box
                        elif rtype == "CYLINDER":
                            N_BEGIN_REP, N_vtx, N_mask, N_joint = N_BEGIN_CYL, N_vtx_cyl, N_mask_cyl, N_joint_cyl
                        else:
                            raise(RuntimeError("Non considered shape key"))    
                        cell = ctem_cells[rtype][ctem_names[rtype].index(rtem.name)]
                        cell, verts, chain = get_cell_data(rtem, L_CELL, Nwdh, Tlink_dict, chain_dict, cell=cell)
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP:N_BEGIN_REP+N_vtx] = verts
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx:N_BEGIN_REP+N_vtx+N_mask] = 1
                        scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask:N_BEGIN_REP+N_vtx+N_mask+N_joint] = chain
                        recover_dat_list.append((rtem.name, rtype, cell, verts, chain))
            print("[END] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
            save_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION.replace("json", "pkl")), action_data_list)
        print("============================ TERMINATE {} ===================================".format(ID))


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




