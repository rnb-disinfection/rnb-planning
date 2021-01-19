import sys
sys.setrecursionlimit(1000000)

from .geometry.ros_rviz import *
from .controller.repeater.repeater import *
from .environment_builder import *
from .sampler.interface import *

from .data_collecting.sampling import *
from .planner.moveit.moveit_planner import gtype_to_otype

PANDA_ROS = False
PORT_REPEATER = 1189
CONTROL_FREQ = 100

PROC_MODE = True
PRINT_LOG = False

##
# @class TMPFramework
# @brief framework for task and motion planning
#
class TMPFramework:

    def __init__(self, ghnd, urdf_path, joint_names, link_names, urdf_content=None, combined_robot=None):
        ## global timer for internal time testing
        self.gtimer=GlobalTimer.instance()

        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        self.ghnd = ghnd
        self.joint_names = joint_names
        self.link_names = link_names
        self.binder_dict = {}
        self.object_binder_dict = defaultdict(list)
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}
        self.joint_limits = np.array([(self.urdf_content.joint_map[jname].limit.lower,
                                       self.urdf_content.joint_map[jname].limit.upper) for jname in self.joint_names])
        self.marker_list = []
        self.combined_robot = combined_robot
        self.planner = None
        self.sampler = None

    def __del__(self):
        try:
            pass
        except: pass

    def set_camera(self, cam):
        self.cam = cam

    #######################################################
    ############## Planner/Sampler ########################
    def set_planner(self, planner):
        self.planner = planner
        planner.update(self)

    def set_sampler(self, sampler):
        self.sampler = sampler
    ############## Planner/Sampler ########################
    #######################################################


    #######################################################
    ##################### Bindings ########################

    ##
    # @brief show motion list(Q in array)
    def show_motion(self, pose_list, from_state=None, **kwargs):
        if from_state is not None:
            self.set_object_state(from_state)
        show_motion(pose_list, self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    def add_binder(self, binder):
        self.binder_dict[binder.name] = binder
        self.object_binder_dict[binder.object.name].append(binder.name)

    def register_binder(self, name, _type, point=None, rpy=(0,0,0), object_name=None, link_name=None):
        if name in self.binder_dict:
            self.remove_binder(name)
        if object_name is None:
            object_name = name

        _object = self.get_object_by_name(object_name)

        if not _object:
            _object = self.ghnd.create_safe(gtype=GEOTYPE.SPHERE, name=name, link_name=link_name,
                                  center=point, dims=(0,0,0), rpy=rpy, collision=False, display=False)
            point=None
            rpy=(0,0,0)

        self.add_binder(_type(name, _object=_object, point=point,rpy=rpy))

    def remove_binder(self, bname):
        self.object_binder_dict[self.binder_dict[bname].object.name].remove(bname)
        del self.binder_dict[bname]

    def get_all_handles(self):
        handles = []
        for obj_hd in self.object_dict.values():
            handles += obj_hd.action_points_dict.values()
        return handles

    def get_all_handle_dict(self):
        handle_dict = {}
        for obj_hd in self.object_dict.values():
            for hd in obj_hd.action_points_dict.values():
                handle_dict[hd.name_full] = hd
        return handle_dict

    def delete_handle(self, htem):
        otem = self.object_dict[htem.object.name]
        del otem.action_points_dict[htem.name]
        if not otem.action_points_dict.keys():
            self.remove_object(htem.object.name)

    @record_time
    def update_handles(self):
        self.handle_dict = {}
        self.handle_list = []
        self.object_list = sorted(self.object_dict.keys())
        for k in self.object_list:
            v = self.object_dict[k]
            ap_list = v.action_points_dict
            self.handle_dict[k] = []
            for ap in ap_list.keys():
                self.handle_dict[k] += [ap]
                self.handle_list += [(k, ap)]

    def get_unique_binders(self):
        uniq_binders = []
        for k_b, binder in self.binder_dict.items():
            if not binder.multiple:
                uniq_binders += [k_b]
        return uniq_binders

    def get_controlled_binders(self):
        controlled_binders = []
        for k_b, binder in self.binder_dict.items():
            if binder.controlled:
                controlled_binders += [k_b]
        return controlled_binders
    #######################################################
    ##################### Bindings ########################

    #######################################################
    ###################### Objects ########################
    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              list2dict([0]*len(self.joint_names), item_names=self.joint_names))

    def remove_object(self, name):
        if name in self.object_dict:
            del self.object_dict[name]

    def register_object(self, name, _type, binding=None, **kwargs):
        if name in self.object_dict:
            self.remove_object(name)

        _object = self.get_object_by_name(name)
        self.object_dict[name] = _type(_object, **kwargs)
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              list2dict([0]*len(self.joint_names), item_names=self.joint_names))

    def register_object_gen(self, objectPose_dict_mv, binder_dict, object_dict, ref_tuple=None, link_name="base_link"):
        object_generators = {k: CallHolder(self.ghnd.create_safe,
                                           ["center", "rpy"], **v.get_geometry_kwargs()) for k, v in
                             self.cam.aruco_map.items() if v.dlevel in [DetectionLevel.MOVABLE, DetectionLevel.ONLINE]}
        if ref_tuple is None:
            ref_tuple = self.cam.ref_tuple
        xyz_rpy_mv_dict, put_point_dict, _ = calc_put_point(self.ghnd, objectPose_dict_mv, self.cam.aruco_map, object_dict, ref_tuple)

        for mname, mgen in object_generators.items():
            if mname in xyz_rpy_mv_dict and mname not in self.ghnd.NAME_DICT:
                xyz_rpy = xyz_rpy_mv_dict[mname]
                mgen(*xyz_rpy, name=mname, link_name=link_name,
                     color=(0.3, 0.3, 0.8, 1), collision=True, fixed=False)

        for bname, bkwargs in binder_dict.items():
            if bname not in self.binder_dict:
                self.register_binder(name=bname, **bkwargs)

        for mtem, xyz_rpy in xyz_rpy_mv_dict.items():
            if mtem in put_point_dict and mtem not in self.object_dict:
                self.register_object(mtem, binding=(put_point_dict[mtem], ref_tuple[0]), **object_dict[mtem])

        return put_point_dict

    def get_object_by_name(self, name):
        if name in self.ghnd.NAME_DICT:
            return self.ghnd.NAME_DICT[name]
        else:
            return None
    ###################### Objects ########################
    #######################################################

    #######################################################
    ##################### Environment #####################
    def set_cam_robot_collision(self, _add_cam_poles = True, color=(0, 1, 0, 0.3), display=True):
        add_geometry_items(self.urdf_content, ghnd=self.ghnd, color=color, display=display, collision=True,
                           exclude_link=["panda1_link7"])
        if _add_cam_poles:
            add_cam_poles(self, self.cam.xyz_rpy_cams)
    ##################### Environment #####################
    #######################################################

    #######################################################
    ######################## State ########################
    def set_object_state(self, state):
        bd_list = list(state.node)
        bd_list_done = []
        while bd_list:
            bd = bd_list.pop(0)
            binder = self.binder_dict[bd[2]]
            if binder.object in [self.object_dict[bd_tmp[0]].object for bd_tmp in bd_list]:
                bd_list += [bd] # prevent using previous info ( move back to end )
            else:
                obj = self.object_dict[bd[0]]
                frame = state.obj_pos_dict[bd[0]]
                binder.link_name = binder.object.link_name # sync linke name with parent
                obj.set_state(frame, binder.link_name, bd[1], bd[2])
                bd_list_done += [bd]

    def get_object_state(self):
        node = ()
        pose_dict = {}
        for k in self.object_list:
            v = self.object_dict[k]
            node += ((k,) + v.binding,)
            pose_dict[k] = v.object.Toff
        return node, pose_dict
    ######################## State ########################
    #######################################################

    #######################################################
    ##################### Transition ######################
    def get_slack_bindings(self, from_state, to_state):
        binding_list = []
        if to_state.node is not None:
            for bd0, bd1 in zip(from_state.node, to_state.node):
                if bd0[2] != bd1[2]: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"
        return binding_list

    def test_transition(self, from_state=None, to_state=None, display=False, dt_vis=1e-2, error_skip=0, **kwargs):
        self.gtimer = GlobalTimer.instance()

        if from_state is not None:
            self.set_object_state(from_state)

        binding_list = self.get_slack_bindings(from_state, to_state)

        success = True
        for binding in binding_list:
            if not self.binder_dict[binding[2]].check_available(list2dict(from_state.Q, self.joint_names)):
                success = False
        if success:
            Traj, LastQ, error, success = self.planner.plan_transition(from_state, to_state, binding_list, **kwargs)
            if display:
                self.show_motion(Traj, from_state, **{'error_skip':error_skip, 'period':dt_vis})
        else:
            print("=====================================================")
            print("=====================================================")
            print("===============Unavailable binder====================")
            print("================={}=======================".format(binding_list))
            print("=====================================================")
            print("=====================================================")
            LastQ = from_state.Q
            Traj = np.array([LastQ])
            error = 1

        if from_state is not None:
            self.set_object_state(from_state)

        if success:
            for bd in binding_list:
                self.rebind(bd, list2dict(LastQ, self.joint_names))

        node, obj_pos_dict = self.get_object_state()
        end_state = State(node, obj_pos_dict, list(LastQ), self)
        return Traj, end_state, error, success

    def get_real_robot_pose(self):
        return self.combined_robot.get_real_robot_pose()

    def execute_grip(self, state):
        grasp_dict = {}
        for name, _type in self.combined_robot.robots_on_scene:
            grasp_dict[name] = False
        for bd in state.node:
            bind_link_name = self.binder_dict[bd[2]].object.link_name
            for name, _type in self.combined_robot.robots_on_scene:
                if name in bind_link_name:
                    grasp_dict[name] = True
        self.combined_robot.grasp_by_dict(grasp_dict)

    def rebind(self, binding, joint_dict_last):
        binder = self.binder_dict[binding[2]]
        object_tar = self.object_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict_last)
        for binder_sub in [k for k,v in self.binder_dict.items() if v.object == object_tar.object]:
            for binding_sub in [(k,v.binding[0]) for k, v in self.object_dict.items()
                                if v.binding[1] == binder_sub]:
                binding_sub += (binder_sub,)
                self.rebind(binding_sub, joint_dict_last)
    ##################### Transition ######################
    #######################################################

    #######################################################
    ############ Visualize - Highlight ####################

    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.object
        Toff_lh = handle.Toff_lh
        axis = "xyz"
        color = None
        self.add_highlight_axis(hl_key, hobj.name, hobj.link_name, Toff_lh[:3,3], Toff_lh[:3,:3], color=color, axis=axis)

    def add_aruco_axis(self, hl_key, atem, axis_name=None):
        oname = atem.oname
        axis_name = axis_name or oname
        if oname in self.combined_robot.get_scene_dict():
            link_name = RobotSpecs.get_base_link(self.combined_robot.get_scene_dict()[oname], oname)
            Toff = atem.Toff
        else:
            aobj = self.ghnd.NAME_DICT[oname]
            link_name = aobj.link_name
            Toff = np.matmul(aobj.Toff, atem.Toff)
        self.add_highlight_axis(hl_key, axis_name, link_name, Toff[:3,3], Toff[:3,:3], axis="xyz")
    ############ Visualize - Highlight ####################
    #######################################################


    #######################################################
    #################### Schedule #########################
    def replay(self, schedule, N=400, dt=0.005,**kwargs):
        state_cur = self.sampler.snode_dict[schedule[0]].state
        for i_state in schedule[1:]:
            snode = self.sampler.snode_dict[i_state]
            state_new = snode.state
            print('')
            print('-'*20)
            print("{}-{}".format(i_state, state_new.node))
            traj, new_state, error, succ = self.test_transition(state_cur, state_new, display=True,
                                                                redundancy_dict=snode.redundancy, N=N, dt=dt,**kwargs)
            try: self.show_pose(traj[-1])
            except: pass
            sleep(0.5)
            state_cur = new_state
        return traj

    def execute_schedule_online(self, snode_schedule, planner, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                on_rviz=False, dynamic_detector=None, rviz_pub=None, stop_count_ref=25,
                                T_step=30, **kwargs):

        state_0 = snode_schedule[0].state
        from_Q = state_0.Q
        object_pose_cur = state_0.obj_pos_dict
        N_step = T_step*control_freq
        if not on_rviz:
            self.execute_grip(state_0)

        for i_s in range(len(snode_schedule) - 1):
            from_state = snode_schedule[i_s].state
            from_state.Q = from_Q
            from_state.obj_pos_dict = object_pose_cur
            to_snode = snode_schedule[i_s + 1]
            to_state = to_snode.state
            traj = to_snode.get_traj()
            redundancy = to_snode.redundancy
            idx_cur = 0
            end_traj = len(traj)-1

            if from_state is not None:
                self.set_object_state(from_state)

            binding_list = self.get_slack_bindings(from_state, to_state)

            pos, binding_list = \
                planner.init_online_plan(from_state, to_state, binding_list, redundancy_dict=redundancy,
                                              control_freq=control_freq, playback_rate=playback_rate,
                                              T_step=T_step, **kwargs
                                              )

            Q0 = np.array(dict2list(pos, self.joint_names))
            if not on_rviz:
                self.combined_robot.joint_make_sure(Q0)
                # print("wait for button input")
                # self.indy.connect_and(self.indy.wait_di, 16)

            stop_count = 0

            with MultiTracker(self.combined_robot.get_robot_list(), self.combined_robot.get_indexing_list(),
                              Q0, on_rviz=on_rviz) as mt:
                time.sleep(0.5)

                i_q = 0

                error_count = 0
                max_err_count = 3
                POS_CUR = from_state.Q
                VEL_CUR = np.zeros_like(POS_CUR)
                end_loop = False
                while True:
                    self.gtimer.tic("move_wait")
                    if on_rviz:
                        self.combined_robot.wait_step(self.rate)
                        all_sent = True
                    else:
                        all_sent = mt.move_possible_joints_x4(POS_CUR)
                    self.gtimer.toc("move_wait", stack=True)

                    if all_sent:
                        if stop_count>0:
                            if stop_count>stop_count_ref:
                                break
                            else:
                                stop_count += 1
                                continue
                        i_q += 1
                        try:
                            self.gtimer.tic("update")
                            obsPos_dict = dynamic_detector.get_dynPos_dict()
                            planner.update_online(obsPos_dict)
                            pos, end_loop, error, success = planner.step_online_plan(i_q, pos, wp_action=idx_cur<end_traj)
                            # print("{i_s} : {idx_cur}<{end_traj}:{wp}".format(i_s=i_s, idx_cur=idx_cur, end_traj=end_traj, wp=idx_cur<end_traj))
                            POS_CUR = np.array(dict2list(pos, self.joint_names))
                            # VEL_CUR = VEL_CUR + e_sim.VEL[i_q, 1::2] * e_sim.DT
                            # POS_CUR = POS_CUR + VEL_CUR * e_sim.DT
                            self.gtimer.toc("update", stack=True)
                        except Exception as e:
                            self.gtimer.tic("error")
                            error_count += 1
                            print("ERROR {}: {}".format(error_count, e))
                            if error_count > max_err_count:
                                print("MAX ERROR REACHED {}".format(error_count))
                                raise (e)
                            POS_CUR = np.array(dict2list(pos, self.joint_names))
                            self.gtimer.toc("error", stack=True)
                        self.gtimer.tic("rviz")
                        if rviz_pub is not None:
                            rviz_pub.update(obsPos_dict, POS_CUR)
                        idx_cur = planner.update_target_joint(idx_cur, traj, POS_CUR)
                        if i_q >= N_step:
                            stop_count+=1
                            continue
                        self.gtimer.toc("rviz", stack=True)
                    if end_loop:
                        stop_count+=1
                        continue
            from_Q = POS_CUR.copy()
            joint_dict = list2dict(POS_CUR, self.joint_names)
            for bd in binding_list:
                self.rebind(bd, joint_dict)
            object_pose_cur = self.get_object_state()[1]
            if success:
                if not on_rviz:
                    self.execute_grip(to_state)
            else:
                print("FAIL ({})".format(error))
                break
    #################### Schedule #########################
    #######################################################

    #######################################################
    ############ Calibration - deprecated #################
    def draw_objects_graph(self, color_image, objectPose_dict, corner_dict, axis_len=0.1):
        return self.cam.aruco_map.draw_objects(color_image, objectPose_dict, corner_dict, self.cam.rs_config[0],
                            self.cam.rs_config[1], axis_len=axis_len)

    def sample_Trel(self, obj_name, obj_link_name, coord_link_name, coord_name, Teo, objectPose_dict_ref):
        aruco_map_new  = {k: self.cam.aruco_map[k] for k in [obj_name, coord_name] if k not in objectPose_dict_ref}
        objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict, objectPoints_dict, point3D_dict, err_dict = \
            get_object_pose_dict_stereo(self.cam.T_c12, self.cam.kn_config, self.cam.rs_config,
                                        aruco_map_new, self.cam.dictionary)
        objectPose_dict.update(objectPose_dict_ref)
        T_co = objectPose_dict[obj_name]
        T_cp = objectPose_dict[coord_name]
        T_p0e = get_tf(obj_link_name, list2dict(self.get_real_robot_pose(), self.joint_names), self.urdf_content,
                       from_link=coord_link_name)
        T_peo = Teo  # SE3(Rot_zyx(0,-np.pi/2,-np.pi/2), [0,0,0.091])
        T_po_cam = np.matmul(SE3_inv(T_cp), T_co)
        T_po_cal = np.matmul(T_p0e, T_peo)
        xyz_cam, rvec_cam = T2xyzrvec(T_po_cam)
        xyz_cal, rvec_cal = T2xyzrvec(T_po_cal)
        return xyz_cam, rvec_cam, xyz_cal, rvec_cal, color_image, objectPose_dict, corner_dict, err_dict
    ############ Calibration - deprecated #################
    #######################################################

    def clear_ws(self):
        ghnd = self.ghnd
        gnames = ghnd.NAME_DICT.keys()
        for gname in gnames:
            self.remove_geometry(ghnd.NAME_DICT[gname])

    def replace_ghnd(self, ghnd_new):
        self.clear_ws()
        for gtem in ghnd_new:
            self.add_marker(self.ghnd.copy_from(gtem))

    def convert_workspace(self, ghnd, L_CELL=0.2, WS_DIMS=(3, 3, 3), center_height=0.75):
        WS_RANGE = np.divide(np.array(WS_DIMS, dtype=np.float), 2)
        WS_RANGE_MIN = -np.copy(WS_RANGE)
        WS_RANGE_MIN[2] = -center_height
        WS_RANGE_MAX = np.copy(WS_RANGE)
        WS_RANGE_MAX[2] = WS_RANGE_MAX[2]*2-center_height
        L_TEM_MAX = L_CELL * 2
        L_REF = ((L_TEM_MAX + L_CELL) / 2)
        ghnd_new = GeometryHandle(ghnd.urdf_content)
        for gtem in ghnd:
            if not gtem.collision:
                continue
            if np.any(np.array(gtem.dims) > L_TEM_MAX):
                if gtem.gtype in [GEOTYPE.CYLINDER, GEOTYPE.CAPSULE]:
                    if gtem.dims[0] > L_TEM_MAX:
                        raise (RuntimeError(
                            "conversion of cylinder diameter larger than max cell size ({}) is not supported yet"))
                    else:  # gtem.dims[2]>L_TEM_MAX:
                        zlen = gtem.dims[2]
                        N_div = int(ceil(zlen / L_REF))
                        zlen_div = zlen / N_div
                        zmin_c, zmax = -zlen / 2 + zlen_div / 2, zlen / 2
                        zval_list = np.arange(zmin_c, zmax, zlen_div)
                        zval = zval_list[0]
                        P_list = [tuple(np.matmul(gtem.orientation_mat, [0, 0, zval])) for zval in zval_list]
                        dims_div = tuple(gtem.dims[:2]) + (zlen_div,)
                        for gidx, P_i in enumerate(P_list):
                            gname_new = gtem.name + "_%03d" % gidx
                            center_new = tuple(np.add(gtem.center, P_i))
                            if np.any([center_new<WS_RANGE_MIN, WS_RANGE_MAX<center_new]):
                                print("ignore {} out of workspace".format(gname_new))
                                continue
                            ghnd_new.create_safe(name=gname_new, link_name=gtem.link_name, gtype=gtem.gtype,
                                                 center=center_new, rpy=gtem.rpy, dims=dims_div, color=gtem.color,
                                                 display=True, collision=True, fixed=gtem.fixed)
                elif gtem.gtype == GEOTYPE.SPHERE:
                    if gtem.dims[0] > L_TEM_MAX:
                        raise (RuntimeError(
                            "conversion of sphere diameter larger than max cell size ({}) is not supported yet"))
                elif gtem.gtype == GEOTYPE.BOX:
                    Ndims = np.ceil(np.divide(gtem.dims, L_REF)).astype(np.int)
                    dims_div = np.divide(gtem.dims, Ndims)
                    P_min = -np.divide(gtem.dims, 2) + dims_div / 2
                    P_list = [tuple(np.matmul(gtem.orientation_mat, P_min + np.multiply(dims_div, (ix, iy, iz)))) for ix
                              in range(Ndims[0]) for iy in range(Ndims[1]) for iz in range(Ndims[2])]
                    for gidx, P_i in enumerate(P_list):
                        gname_new = gtem.name + "_%03d" % gidx
                        center_new = tuple(np.add(gtem.center, P_i))
                        if np.any([center_new<WS_RANGE_MIN, WS_RANGE_MAX<center_new]):
                            print("ignore {} out of workspace".format(gname_new))
                            print(center_new)
                            continue
                        ghnd_new.create_safe(name=gname_new, link_name=gtem.link_name, gtype=gtem.gtype,
                                             center=center_new, rpy=gtem.rpy, dims=tuple(dims_div), color=gtem.color,
                                             display=True, collision=True, fixed=gtem.fixed)
            else:
                if np.any([gtem.center<WS_RANGE_MIN, WS_RANGE_MAX<gtem.center]):
                    print("ignore {} out of workspace".format(gtem.name))
                    continue
                ghnd_new.create_safe(name=gtem.name, link_name=gtem.link_name, gtype=gtem.gtype,
                                     center=gtem.center, rpy=gtem.rpy, dims=gtem.dims,
                                     color=gtem.color, display=True, collision=True, fixed=gtem.fixed)
        return ghnd_new

    def convert_scene(self, ghnd_cvt, state, L_CELL=0.2, Nwdh=(15, 15, 15), BASE_LINK="base_link", offset_center=True,
                      only_base=True, center_height=0.75):
        crob = self.combined_robot
        gtimer = self.gtimer
        Q_s = state.Q
        Q_s_dict = list2dict(Q_s, self.joint_names)
        self.set_object_state(state)

        link_names = self.link_names
        joint_names = crob.joint_names
        joint_num = crob.joint_num
        IGNORE_CTEMS = ["panda1_hand_Cylinder_1", 'panda1_hand_Cylinder_2']
        joint_index_dict = {joint.name: None for joint in ghnd_cvt.urdf_content.joints}
        joint_index_dict.update({jname: idx for idx, jname in zip(range(joint_num), joint_names)})
        centers = get_centers(Nwdh, L_CELL)
        merge_pairs = get_merge_pairs(ghnd_cvt, BASE_LINK)
        merge_paired_ctems(ghnd=ghnd_cvt, merge_pairs=merge_pairs, VISUALIZE=False)
        for cname in IGNORE_CTEMS:
            if cname in ghnd_cvt.NAME_DICT:
                ghnd_cvt.remove(ghnd_cvt.NAME_DICT[cname])

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

        N_joint_label_begin = N_label_box + N_label_cyl + N_label_init + N_label_goal

        N_joint_label = 6 * joint_num

        N_joint_limits = 3 * joint_num

        N_cell_label = N_joint_label_begin + N_joint_label + N_joint_limits

        gtimer.tic("test_links")
        Tlink_dict = {}
        chain_dict = {}
        Tj_arr = np.zeros((joint_num, 4, 4))
        Tj_inv_arr = np.zeros((joint_num, 4, 4))
        joint_axis_arr = np.zeros((joint_num, 3))

        gtimer.tic("T_chain")
        Tlink_dict = build_T_chain(link_names, Q_s_dict, ghnd_cvt.urdf_content)
        if offset_center:
            Toffcenter = SE3(np.identity(3), tuple(np.multiply(Nwdh[:2], L_CELL) / 2)+(center_height,))
            for key in Tlink_dict.keys():
                Tlink_dict[key] = np.matmul(Toffcenter, Tlink_dict[key])
        gtimer.toc("T_chain")

        for lname in link_names:
            gtimer.tic("get_chain")
            jnames = filter(lambda x: x in joint_names,
                            ghnd_cvt.urdf_content.get_chain(root=BASE_LINK, tip=lname, joints=True, links=False))
            gtimer.toc("get_chain")
            chain_dict[lname] = [1 if pj in jnames else 0 for pj in joint_names]
        gtimer.toc("test_links")

        gtimer.tic("test_joints")
        for i_j, jname in zip(range(joint_num), joint_names):
            joint = ghnd_cvt.urdf_content.joint_map[jname]
            lname = joint.child
            Tj_arr[i_j, :, :] = Tlink_dict[lname]
            Tj_inv_arr[i_j, :, :] = SE3_inv(Tlink_dict[lname])
            joint_axis_arr[i_j] = joint.axis
        gtimer.toc("test_joints")
        gtimer.tic("calc_cell")
        __p = centers.reshape(Nwdh + (1, 3)) - Tj_arr[:, :3, 3].reshape((1, 1, 1, joint_num, 3))
        __w = np.sum(Tj_arr[:, :3, :3] * joint_axis_arr.reshape(joint_num, 1, 3), axis=-1).reshape(
            (1, 1, 1, joint_num, 3))
        __w = np.tile(__w, Nwdh + (1, 1))
        __v = np.cross(__w, __p)
        xi = np.concatenate([__w, __v], axis=-1)
        gtimer.toc("calc_cell")
        gtimer.tic("list_ctems")
        ctem_names, ctem_TFs, ctem_dims, ctem_cells, ctem_links, ctem_joints, ctem_types = \
            defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(
                list), dict()
        for ctem in ghnd_cvt:
            if not ctem.collision:
                continue
            key = gtype_to_otype(ctem.gtype).name
            ctem_names[key].append(ctem.name)
            Ttem = np.matmul(Tlink_dict[ctem.link_name], ctem.Toff)
            ctem_TFs[key].append(Ttem)
            ctem_dims[key].append(ctem.dims)
            ctem_cells[key].append(get_cell(Ttem[:3, 3], L_CELL, Nwdh))
            ctem_links[key].append(ctem.link_name)
            ctem_joints[key].append(chain_dict[ctem.link_name])
            ctem_types[key] = ctem.gtype
        gtimer.toc("list_ctems")

        gtimer.tic("calc_ctems")

        verts_dict = {}
        for key_cur in ctem_cells:
            ctem_TFs[key_cur] = np.array(ctem_TFs[key_cur])
            cell_array = np.array(ctem_cells[key_cur])
            all_rearranged = False
            while not all_rearranged:
                all_rearranged = True
                for cell in cell_array:
                    idxset = np.where(np.all(cell_array == cell, axis=-1))[0]
                    if len(idxset) > 1:
                        all_rearranged = False
                        cell_array_bak = cell_array.copy()
                        gtimer.tic("rearrange_cell_array")
                        cell_array = rearrange_cell_array_fast(cell_array, idxset, L_CELL, Nwdh, ctem_TFs[key_cur],
                                                               centers)
                        gtimer.toc("rearrange_cell_array")
                        break
            ctem_cells[key_cur] = cell_array
            gtype = ctem_types[key_cur]
            TFs = ctem_TFs[key_cur]
            dims = np.array(ctem_dims[key_cur])
            default_vert = DEFAULT_VERT_DICT[gtype]
            verts_dim = default_vert.reshape((1, -1, 3)) * dims.reshape((-1, 1, 3))
            cell_array = ctem_cells[key_cur]
            verts = np.zeros_like(verts_dim)
            for iv in range(verts.shape[1]):
                verts[:, iv, :] = matmul_md(TFs[:, :3, :3], verts_dim[:, iv, :, np.newaxis])[:, :, 0] + TFs[:, :3, 3]
            verts_loc = (verts - (cell_array[:, np.newaxis, :] * L_CELL + L_CELL / 2))
            verts_loc = verts_loc.reshape((verts_loc.shape[0], -1))
            if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
                verts_loc = np.concatenate([verts_loc, dims[:, 0:1]], axis=-1)
            verts_dict[key_cur] = verts_loc
        gtimer.toc("calc_ctems")

        ### initialize data volume
        scene_data = np.zeros(Nwdh + (N_cell_label,))
        ### put cell joint data
        scene_data[:, :, :, N_joint_label_begin:N_joint_label_begin + N_joint_label] = xi.reshape(
            Nwdh + (N_joint_label,))

        jtem_list = [ghnd_cvt.urdf_content.joint_map[jname] for jname in self.joint_names]
        joint_limits = [jtem.limit.lower for jtem in jtem_list] + [jtem.limit.upper for jtem in jtem_list]
        scene_data[:, :, :, -N_joint_limits:] = Q_s.tolist() + joint_limits
        ### put cell item data
        for verts, cell, chain in zip(verts_dict["BOX"], ctem_cells["BOX"], ctem_joints["BOX"]):
            scene_data[cell[0], cell[1], cell[2], :N_vtx_box] = verts
            scene_data[cell[0], cell[1], cell[2], N_vtx_box:N_vtx_box + N_mask_box] = 1
            scene_data[cell[0], cell[1], cell[2], N_vtx_box + N_mask_box:N_vtx_box + N_mask_box + N_joint_box] = chain

        N_BEGIN_CYL = N_vtx_box + N_mask_box + N_joint_box
        for verts, cell, chain in zip(verts_dict["CYLINDER"], ctem_cells["CYLINDER"], ctem_joints["CYLINDER"]):
            scene_data[cell[0], cell[1], cell[2], N_BEGIN_CYL:N_BEGIN_CYL + N_vtx_cyl] = verts
            scene_data[cell[0], cell[1], cell[2], N_BEGIN_CYL + N_vtx_cyl:N_BEGIN_CYL + N_vtx_cyl + N_mask_cyl] = 1
            scene_data[cell[0], cell[1], cell[2],
            N_BEGIN_CYL + N_vtx_cyl + N_mask_cyl:N_BEGIN_CYL + N_vtx_cyl + N_mask_cyl + N_joint_cyl] = chain
        return scene_data, ctem_names, ctem_cells, chain_dict
        # save_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, "scene.pkl"), {"scene_data": scene_data, "ctem_names": ctem_names, "ctem_cells":ctem_cells})