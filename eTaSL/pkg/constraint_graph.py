import sys
sys.setrecursionlimit(1000000)

from .geometry.ros_rviz import *
from urdf_parser_py.urdf import URDF
from .controller.repeater import *
from .environment_builder import *
from .sampler.interface import *

PANDA_ROS = False
PORT_REPEATER = 1189
CONTROL_FREQ = 100

PROC_MODE = True
PRINT_LOG = False


class ConstraintGraph:

    def __init__(self, urdf_path, joint_names, link_names, urdf_content=None, combined_robot=None):
        self.joint_num = len(joint_names)
        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        self.ghnd = GeometryHandle.instance()
        self.ghnd.set_urdf_content(urdf_content)
        set_parent_joint_map(urdf_content)
        set_link_adjacency_map(urdf_content)
        self.min_distance_map = set_min_distance_map(link_names, urdf_content)
        self.joint_names = joint_names
        self.link_names = link_names
        self.binder_dict = {}
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}
        self.gtimer=GlobalTimer.instance()
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
    ############ Visualize - Geometry #####################
    def set_rviz(self, joint_pose=None):
        # prepare ros
        if not (hasattr(self, 'pub') and hasattr(self, 'joints') and hasattr(self, 'rate')):
            self.pub, self.joints, self.rate = get_publisher(self.joint_names, control_freq=CONTROL_FREQ)
        if joint_pose is None:
            joint_pose = self.joints.position
        # prepare visualization markers
        self.clear_markers()
        self.marker_list = set_markers(self.ghnd, self.joints, self.joint_names)
        self.show_pose(joint_pose)
        self.show_pose(joint_pose)

    def show_pose(self, pose, **kwargs):
        show_motion([pose], self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    def show_motion(self, pose_list, from_state=None, **kwargs):
        if from_state is not None:
            self.set_object_state(from_state)
        show_motion(pose_list, self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    def remove_geometry(self, gtem, from_ghnd=True):
        del_list = []
        for marker in self.marker_list:
            if marker.geometry == gtem:
                del_list.append(marker)
        for marker in del_list:
            marker.delete()
            self.marker_list.remove(marker)

        if from_ghnd:
            self.ghnd.remove(gtem)

    def add_geometry(self, gtem):
        self.marker_list += set_markers([gtem], self.joints, self.joint_names)

    def clear_markers(self):
        for mk in self.marker_list:
            mk.delete()
        self.marker_list = []
        if hasattr(self, 'highlight_dict'):
            for hkey, hset in self.highlight_dict.items():
                for k,v in hset.items():
                    self.remove_geometry(v)
                    del self.highlight_dict[hkey][k]
        else:
            self.highlight_dict = defaultdict(lambda: dict())

    def update_marker(self, gtem):
        joint_dict = {self.joints.name[i]: self.joints.position[i] for i in range(len(self.joint_names))}
        marks = [mk for mk in self.marker_list if mk.geometry == gtem]
        for mk in marks:
            mk.set_marker(joint_dict, create=False)
        return marks
    ############ Visualize - Geometry #####################
    #######################################################

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
    def add_binder(self, name, binder):
        self.binder_dict[name] = binder

    def register_binder(self, name, _type, link_name=None, object_name=None, **kwargs):
        if name in self.binder_dict:
            self.remove_binder(name)
        if object_name is None:
            object_name = name

        _object = self.get_object_by_name(object_name)

        if _object:
            link_name = _object.link_name
        else:
            _object=None
            assert link_name is not None, "The object should be registered first or give link name"
            assert "point" in kwargs, "The object should be registered first or give interaction point"
        self.binder_dict[name] = _type(_object=_object,
                                       name=name, link_name=link_name,
                                       urdf_content=self.urdf_content, **kwargs)

    def remove_binder(self, bname):
        del self.binder_dict[bname]

    def get_all_handles(self):
        handles = []
        for obj_hd in self.object_dict.values():
            handles += obj_hd.get_action_points().values()
        return handles

    def get_all_handle_dict(self):
        handle_dict = {}
        for obj_hd in self.object_dict.values():
            for hd in obj_hd.get_action_points().values():
                handle_dict[hd.name_constraint] = hd
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
            ap_list = v.get_action_points()
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
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

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
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

    def register_object_gen(self, objectPose_dict_mv, binder_dict, object_dict, ref_tuple=None, link_name="world"):
        object_generators = {k: CallHolder(GeometryHandle.instance().create_safe,
                                           ["center", "rpy"], **v.get_kwargs()) for k, v in
                             self.cam.aruco_map.items() if v.ttype in [TargetType.MOVABLE, TargetType.ONLINE]}
        if ref_tuple is None:
            ref_tuple = self.cam.ref_tuple
        xyz_rpy_mv_dict, put_point_dict, _ = calc_put_point(objectPose_dict_mv, self.cam.aruco_map, object_dict, ref_tuple)

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
    def set_cam_robot_collision(self):
        add_geometry_items(self.urdf_content, color=(0, 1, 0, 0.3), display=True, collision=True,
                           exclude_link=["panda1_link7"])
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
            pose_dict[k] = v.object.get_frame()
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
            if not self.binder_dict[binding[2]].check_available(joint_list2dict(from_state.Q, self.joint_names)):
                success = False
        if success:
            Traj, LastQ, error, success = self.planner.plan_transition(from_state, to_state, binding_list, **kwargs)
            if display:
                self.show_motion(Traj, from_state, **{'error_skip':error_skip, 'period':dt_vis})
        else:
            print("=====================================================")
            print("=====================================================")
            print("=====================================================")
            print("===============Unavailable binder====================")
            print("=====================================================")
            print("=====================================================")
            print("=====================================================")
            LastQ = from_state.Q
            Traj = np.array([LastQ])
            error = 1

        if from_state is not None:
            self.set_object_state(from_state)

        if success:
            for bd in binding_list:
                self.rebind(bd, joint_list2dict(LastQ, self.joint_names))

        node, obj_pos_dict = self.get_object_state()
        end_state = State(node, obj_pos_dict, list(LastQ))
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
    def clear_highlight(self, hl_keys=[]):
        for hl_key, hl_set in self.highlight_dict.items():
            if hl_key in hl_keys or not hl_keys:
                for k,v in hl_set.items():
                    self.remove_geometry(v)
                    del self.highlight_dict[hl_key][k]

    def highlight_geometry(self, hl_key, gname, color=(1, 0.3, 0.3, 0.5)):
        if gname not in self.ghnd.NAME_DICT:
            return
        gtem = self.ghnd.NAME_DICT[gname]
        dims = gtem.dims if np.sum(gtem.dims) > 0.001 else (0.03, 0.03, 0.03)
        hname = "hl_" + gtem.name
        if hname in self.ghnd.NAME_DICT:
            return
        htem = self.ghnd.create_safe(gtype=gtem.gtype, name=hname, link_name=gtem.link_name,
                            center=gtem.center, dims=dims, rpy=Rot2rpy(gtem.orientation_mat), color=color,
                            collision=False)

        self.highlight_dict[hl_key][htem.name] = htem
        self.add_geometry(htem)

    def add_highlight_axis(self, hl_key, name, link_name, center, orientation_mat, color=None, axis="xyz", dims=(0.10, 0.01, 0.01)):
        if 'x' in axis:
            axtemx = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axx_" + name, link_name=link_name,
                                  center=center, dims=dims, rpy=Rot2rpy(orientation_mat), color=color or (1, 0, 0, 0.5),
                                  collision=False)
            self.add_geometry(axtemx)
            self.highlight_dict[hl_key][axtemx.name] = axtemx

        if 'y' in axis:
            axtemy = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axy_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(3, np.pi / 2))), color=color or (0, 1, 0, 0.5),
                                  collision=False)
            self.add_geometry(axtemy)
            self.highlight_dict[hl_key][axtemy.name] = axtemy

        if 'z' in axis:
            axtemz = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axz_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(2, -np.pi / 2))),
                                  color=color or (0, 0, 1, 0.5),
                                  collision=False)
            self.add_geometry(axtemz)
            self.highlight_dict[hl_key][axtemz.name] = axtemz

    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.handle.object
        if hasattr(handle, 'orientation_mat'):
            orientation_mat = np.matmul(hobj.orientation_mat, handle.orientation_mat)
            axis = "xyz"
            color = None
        elif hasattr(handle, 'direction'):
            orientation_mat = np.matmul(hobj.orientation_mat,
                                        Rotation.from_rotvec(calc_rotvec_vecs([1, 0, 0], handle.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.add_highlight_axis(hl_key, hobj.name, hobj.link_name, hobj.center, orientation_mat, color=color, axis=axis)

    def add_binder_axis(self, hl_key, binder, color=None):
        bobj = binder.effector.object
        if hasattr(binder, 'orientation'):
            orientation_mat = np.matmul(bobj.orientation_mat, binder.effector.orientation_mat)
            axis = "xyz"
        elif hasattr(binder, 'direction'):
            orientation_mat = np.matmul(bobj.orientation_mat, Rotation.from_rotvec(
                calc_rotvec_vecs([1, 0, 0], binder.effector.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.add_highlight_axis(hl_key, bobj.name, bobj.link_name, bobj.center, orientation_mat, color=color, axis=axis)

    def add_aruco_axis(self, hl_key, atem, axis_name=None):
        oname = atem.oname
        axis_name = axis_name or oname
        if oname in self.combined_robot.get_scene_dict():
            link_name = RobotType.get_base_link(self.combined_robot.get_scene_dict()[oname], oname)
            Toff = atem.Toff
        else:
            aobj = self.ghnd.NAME_DICT[oname]
            link_name = aobj.link_name
            Toff = np.matmul(aobj.get_offset_tf(), atem.Toff)
        self.add_highlight_axis(hl_key, axis_name, link_name, Toff[:3,3], Toff[:3,:3], axis="xyz")
    ############ Visualize - Highlight ####################
    #######################################################


    #######################################################
    #################### Schedule #########################
    def replay(self, schedule, N=400, dt=0.005,**kwargs):
        state_cur = self.sampler.snode_dict[schedule[0]].state
        for i_state in schedule[1:]:
            state_new = self.sampler.snode_dict[i_state].state
            print('')
            print('-'*20)
            print("{}-{}".format(i_state, state_new.node))
            traj, new_state, error, succ = self.test_transition(state_cur, state_new, display=True, N=N, dt=dt,**kwargs)
            state_cur = state_new
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
            idx_cur = 0
            end_traj = len(traj)-1

            if from_state is not None:
                self.set_object_state(from_state)

            binding_list = self.get_slack_bindings(from_state, to_state)

            pos, binding_list = \
                planner.init_online_plan(from_state, to_state, binding_list,
                                              control_freq=control_freq, playback_rate=playback_rate,
                                              T_step=T_step, **kwargs
                                              )

            Q0 = np.array(joint_dict2list(pos, self.joint_names))
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
                    if on_rviz:
                        self.combined_robot.wait_step(self.rate)
                        all_sent = True
                    else:
                        all_sent = mt.move_possible_joints_x4(POS_CUR)

                    if all_sent:
                        if stop_count>0:
                            if stop_count>stop_count_ref:
                                break
                            else:
                                stop_count += 1
                                continue
                        i_q += 1
                        try:
                            obsPos_dict = dynamic_detector.get_dynPos_dict()
                            planner.update_online(obsPos_dict)
                            pos, end_loop, error, success = planner.step_online_plan(i_q, pos, wp_action=idx_cur<end_traj)
                            # print("{i_s} : {idx_cur}<{end_traj}:{wp}".format(i_s=i_s, idx_cur=idx_cur, end_traj=end_traj, wp=idx_cur<end_traj))
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                            # VEL_CUR = VEL_CUR + e_sim.VEL[i_q, 1::2] * e_sim.DT
                            # POS_CUR = POS_CUR + VEL_CUR * e_sim.DT
                        except Exception as e:
                            error_count += 1
                            print("ERROR {}: {}".format(error_count, e))
                            if error_count > max_err_count:
                                print("MAX ERROR REACHED {}".format(error_count))
                                raise (e)
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                        if rviz_pub is not None:
                            rviz_pub.update(obsPos_dict, POS_CUR)
                        idx_cur = planner.update_target_joint(idx_cur, traj, POS_CUR)
                        if i_q >= N_step:
                            stop_count+=1
                            continue
                    if end_loop:
                        stop_count+=1
                        continue
            from_Q = POS_CUR.copy()
            joint_dict = joint_list2dict(POS_CUR, self.joint_names)
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
        return draw_objects(color_image, self.cam.aruco_map, objectPose_dict, corner_dict, self.cam.rs_config[0],
                            self.cam.rs_config[1], axis_len=axis_len)

    def sample_Trel(self, obj_name, obj_link_name, coord_link_name, coord_name, Teo, objectPose_dict_ref):
        aruco_map_new  = {k: self.cam.aruco_map[k] for k in [obj_name, coord_name] if k not in objectPose_dict_ref}
        objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict, objectPoints_dict, point3D_dict, err_dict = \
            get_object_pose_dict_stereo(self.cam.T_c12, self.cam.kn_config, self.cam.rs_config,
                                        aruco_map_new, self.cam.dictionary)
        objectPose_dict.update(objectPose_dict_ref)
        T_co = objectPose_dict[obj_name]
        T_cp = objectPose_dict[coord_name]
        T_p0e = get_tf(obj_link_name, joint_list2dict(self.get_real_robot_pose(), self.joint_names), self.urdf_content,
                       from_link=coord_link_name)
        T_peo = Teo  # SE3(Rot_zyx(0,-np.pi/2,-np.pi/2), [0,0,0.091])
        T_po_cam = np.matmul(SE3_inv(T_cp), T_co)
        T_po_cal = np.matmul(T_p0e, T_peo)
        xyz_cam, rvec_cam = T2xyzrvec(T_po_cam)
        xyz_cal, rvec_cal = T2xyzrvec(T_po_cal)
        return xyz_cam, rvec_cam, xyz_cal, rvec_cal, color_image, objectPose_dict, corner_dict, err_dict
    ############ Calibration - deprecated #################
    #######################################################