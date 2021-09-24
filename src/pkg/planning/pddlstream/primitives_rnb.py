import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from ...utils.rotation_utils import *
from ...utils.joint_utils import *
from ...utils.utils import *
from ..constraint.constraint_common import *
from primitives_pybullet import *
from examples.pybullet.tamp.streams import *
from constants_common import *
from pkg.planning.scene import State

TIMEOUT_MOTION_DEFAULT = 1

DEBUG_MODE_PRIM_RNB = False
GLOBAL_LOG_ENABLED = True
POSTPONE_DEFAULT = 5.0
DT_DEFAULT = 1

if DEBUG_MODE_PRIM_RNB:
    TextColors.RED.println("===== WARNING: primitives_rnb in DEBUG MODE====")

gtimer= GlobalTimer.instance()
eye4 = np.identity(4)


from itertools import count


class Time(object):
    num = count()
    def __init__(self, t):
        self.t = t
        self.index = next(self.num)
    @property
    def value(self):
        return self.t

    def __repr__(self):
        index = self.index
        return 'i{}'.format(index)

def get_time_gen(dt=DT_DEFAULT):
    def gen(body):
        while True:
            _t = time.time()
            if _t - gen.time_last> dt:
                gen.time_last = _t
                time_ = Time(_t)
                yield (time_,)
            else:
                yield None
    gen.time_last = 0
    return gen

##
# @brief find matching object at a given configuration to the given binder. Assume the handle is empty.
def get_matching_object(pscene, binder, approach_vec, Q_dict):
    margin_max = -1e10
    max_point = ""
    max_binder = ""

    binder_T = binder.get_tf_handle(Q_dict)

    if approach_vec is not None:
        binder_link_T = np.matmul(binder_T, SE3_inv(binder.Toff_lh))
        binder_link_T_approached = np.matmul(binder_link_T, SE3(np.identity(3), np.negative(approach_vec)))
        binder_T = np.matmul(binder_link_T_approached, binder.Toff_lh)


    for sname, subject in pscene.subject_dict.items():
        self_family = subject.geometry.get_family()
        ## find best binding between object and binders
        for kpt, handle in subject.action_points_dict.items():
            handle_T = handle.get_tf_handle(Q_dict)

            if binder.check_available(Q_dict):
                if binder.geometry.name in self_family or not binder.check_type(handle):
                    continue
                binder_redundancy = binder.get_redundancy()
                handle_redundancy = handle.get_redundancy()
                margins = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy,
                                              rot_scale=1e-2)
                margin_min = np.min(margins) # get max. deviation (negative)
                if margin_min > margin_max: # pick smallest max. deviation (negative)
                    margin_max = margin_min
                    max_point = handle.name
                    max_subject = subject
    if margin_max < -1e-2:
        return None
    return max_subject

##
# @brief    find matching binder at a given configuration to the given subject.
#           Assume the object is held by the robot and we are looking for a place to go.
# @param    excludes    give the currently holding actor to exclude from the target candidates that the object is going to.
def get_matching_binder(pscene, subject, Q_dict, excludes=[]):
    margin_max = -1e10
    max_point = ""
    max_binder = ""
    actor_dict = {bname: binder for bname, binder in pscene.actor_dict.items()
                   if bname not in excludes and binder not in excludes}
    binder_T_dict = {bname: binder.get_tf_handle(Q_dict) for bname, binder in actor_dict.items()}
    self_family = subject.geometry.get_family()
    ## find best binding between object and binders
    for kpt, handle in subject.action_points_dict.items():
        handle_T = handle.get_tf_handle(Q_dict)

        for bname, binder in actor_dict.items():
            if binder.check_available(Q_dict):
                binder_T = binder_T_dict[bname]
                if binder.geometry.name in self_family or not binder.check_type(handle):
                    continue
                binder_redundancy = binder.get_redundancy()
                handle_redundancy = handle.get_redundancy()
                margins = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy,
                                              rot_scale=1e-2)
                margin_min = np.min(margins)
                if margin_min > margin_max:
                    margin_max = margin_min
                    max_point = handle.name
                    max_binder = binder
    return max_binder


def plan_motion(mplan, body_subject_map, conf1, conf2, grasp, fluents, tool, tool_link, approach_vec=None,
                timeout=TIMEOUT_MOTION_DEFAULT, base_link="base_link", skip_feas=False, show_state=False):
    pscene = mplan.pscene
    for fluent in fluents:
        subject = body_subject_map[fluent[1]]
        Tbase = T_xyzquat(fluent[2].value)
        subject.set_state(binding=(subject.oname, None, None, None),
                          state_param=(base_link, Tbase))

    if grasp is not None:
        graspped = body_subject_map[grasp.body]
        Tgrasp = T_xyzquat(grasp.grasp_pose)
        graspped.set_state(binding=(graspped.oname, None, tool.geometry.name, tool.name),
                          state_param=(tool_link, Tgrasp))

    Qcur = conf1.values
    # from_state = pscene.initialize_state(np.array(Qcur))   #This resets the binding state
    binding_state, state_param = pscene.get_object_state()
    from_state = State(binding_state, state_param, list(Qcur), pscene)
    Qfrom_dict = list2dict(Qcur, pscene.gscene.joint_names)
    if show_state:
        pscene.gscene.clear_highlight()
        pscene.gscene.update_markers_all()


    Qto = conf2.values
    to_state = from_state.copy(pscene)
    to_state.Q = np.array(Qto)
    Qto_dict = list2dict(Qto, pscene.gscene.joint_names)

    subject_from = None
    actor_from = None
    if grasp is None: # free motion, go to grasp - actor:tool
        subject_from = get_matching_object(pscene,tool, approach_vec, Qfrom_dict)
        subject = get_matching_object(pscene,tool, approach_vec, Qto_dict)
        actor = tool
    else: # holding motion, go to release - actor:plane
        subject = graspped
        actor_from = get_matching_binder(pscene, subject, Qfrom_dict, excludes=[tool])
        actor = get_matching_binder(pscene, subject, Qto_dict, excludes=[tool])
    if show_state:
        print("sucject/actor: {} / {}".format(subject.oname if subject is not None else subject, actor.name))

    # GlobalLogger.instance()["from_state"] = from_state
    # GlobalLogger.instance()["to_state"] = to_state
    # GlobalLogger.instance()["subject"] = subject.oname if hasattr(subject, "oname") else subject
    # GlobalLogger.instance()["actor"] = actor.name if hasattr(actor, "name") else actor
    ## Check motion_filters outside, as plan_transition below will do joint motion - filters will be skipped
    feas = True
    if skip_feas:
        feas = True
    else:
        if subject is not None:
            ckey = get_checker_keys(tool, subject, fluents, conf2, body_subject_map)
            if DEBUG_MODE_PRIM_RNB: print(ckey)
            if ckey in run_checkers.cache:
                feas = run_checkers.cache[ckey]
                if DEBUG_MODE_PRIM_RNB:
                    Tloal_list = [
                        get_tf(to_link=actor.geometry.link_name, from_link=subject.geometry.link_name,
                               joint_dict=Qto_dict, urdf_content=pscene.gscene.urdf_content)]
                    newres = run_checkers(mplan.motion_filters, actor, subject, Tloal_list,
                                            Q_dict=list2dict(Qcur, pscene.gscene.joint_names), show_state=show_state, mplan=mplan)
                    mplan.result_log["cache_log"].append(
                        (feas, newres)
                    )
                    feas = newres
            else:
                Tloal_list = [
                    get_tf(to_link=actor.geometry.link_name, from_link=subject.geometry.link_name,
                           joint_dict=Qto_dict, urdf_content=pscene.gscene.urdf_content)]
                feas = run_checkers(mplan.motion_filters, actor, subject, Tloal_list,
                             Q_dict=list2dict(Qcur, pscene.gscene.joint_names), show_state=show_state, mplan=mplan)
                run_checkers.cache[ckey] = feas
        if feas and subject_from is not None:
            ckey = get_checker_keys(tool, subject_from, fluents, conf1, body_subject_map)
            if DEBUG_MODE_PRIM_RNB: print(ckey)
            if ckey in run_checkers.cache:
                feas = run_checkers.cache[ckey]
                if DEBUG_MODE_PRIM_RNB:
                    Tloal_list = [
                        get_tf(to_link=actor.geometry.link_name, from_link=subject_from.geometry.link_name,
                               joint_dict=Qfrom_dict, urdf_content=pscene.gscene.urdf_content)]
                    newres = run_checkers(mplan.motion_filters, actor, subject_from, Tloal_list,
                                 Q_dict=Qto_dict, show_state=show_state, mplan=mplan)
                    mplan.result_log["cache_log"].append(
                        (feas, newres)
                    )
                    feas = newres
            else:
                Tloal_list = [
                    get_tf(to_link=actor.geometry.link_name, from_link=subject_from.geometry.link_name,
                           joint_dict=Qfrom_dict, urdf_content=pscene.gscene.urdf_content)]
                feas = run_checkers(mplan.motion_filters, actor, subject_from, Tloal_list,
                             Q_dict=Qto_dict, show_state=show_state, mplan=mplan)
                run_checkers.cache[ckey] = feas

        if feas and actor_from is not None:
            ckey = get_checker_keys(tool, subject, fluents, conf1, body_subject_map)
            if DEBUG_MODE_PRIM_RNB: print(ckey)
            if ckey in run_checkers.cache:
                feas = run_checkers.cache[ckey]
                if DEBUG_MODE_PRIM_RNB:
                    Tloal_list = [
                        get_tf(to_link=actor_from.geometry.link_name, from_link=subject.geometry.link_name,
                               joint_dict=Qfrom_dict, urdf_content=pscene.gscene.urdf_content)]
                    newres = run_checkers(mplan.motion_filters, actor_from, subject, Tloal_list,
                                            Q_dict=Qto_dict, show_state=show_state, mplan=mplan)
                    mplan.result_log["cache_log"].append(
                        (feas, newres)
                    )
                    feas = newres
            else:
                Tloal_list = [
                    get_tf(to_link=actor_from.geometry.link_name, from_link=subject.geometry.link_name,
                           joint_dict=Qfrom_dict, urdf_content=pscene.gscene.urdf_content)]
                feas = run_checkers(mplan.motion_filters, actor_from, subject, Tloal_list,
                             Q_dict=Qto_dict, show_state=show_state, mplan=mplan)
                run_checkers.cache[ckey] = feas

    if mplan.flag_log:
        mplan.result_log["pre_motion_checks"].append(feas)

    if feas or DEBUG_MODE_PRIM_RNB:
        Traj, LastQ, error, success, binding_list = mplan.plan_transition(
            from_state, to_state, timeout=timeout, show_state=show_state)
    else:
        Traj, success = [], False

    if show_state:
        print("check / plan: {} / {}".format(feas, success))
        pscene.gscene.show_motion(Traj)

    return Traj, success, feas

def get_free_motion_gen_rnb(mplan, body_subject_map, robot, tool, tool_link, approach_vec, timeout=TIMEOUT_MOTION_DEFAULT,
                            base_link="base_link", show_state=False, POSTPONE=POSTPONE_DEFAULT):
    time_dict = {}
    returneds = set()
    def fn(conf1, conf2, time_=None, fluents=[]):
        tkey = (conf1.index, conf2.index) + tuple([(fl[0], fl[1], fl[2].index) for fl in fluents])
        if tkey in returneds:
            return None

        skip_feas = False
        if time_ is not None:
            firstcall = tkey not in time_dict
            if not firstcall:
                skip_feas = time_.value - time_dict[tkey] > POSTPONE

        if (time_ is None # not timed domain (do feas-motion)
                or firstcall # first call in timed domain (do feas-motion)
                or skip_feas): # postpone time passed (do motion only)
            with GlobalTimer.instance().block("free_motion_gen"):
                assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
                conf1.assign()
                path, succ, feas = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                         conf1=conf1, conf2=conf2, grasp=None, fluents=fluents, tool=tool,
                                         tool_link=tool_link, base_link=base_link, timeout=timeout,
                                         show_state=show_state, approach_vec=approach_vec,
                                         skip_feas=skip_feas)

                if succ or skip_feas:
                    returneds.add(tkey) # success returned on first call or postpone time passed - mark finished case
                if not succ:
                    if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                    return None
                command = Command([BodyPath(robot, path, joints=conf2.joints)])
                return (command,)
        else: # non-first call, postpone time not passed (X)
            return None
    return fn


def get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool, tool_link, approach_vec, timeout=TIMEOUT_MOTION_DEFAULT,
                               base_link="base_link", show_state=False, POSTPONE=POSTPONE_DEFAULT):
    time_dict = {}
    returneds = set()
    def fn(conf1, conf2, body, grasp, time_=None, fluents=[]):
        tkey = (conf1.index, conf2.index, body, grasp.index) + tuple([(fl[0], fl[1], fl[2].index) for fl in fluents])
        if tkey in returneds:
            return None

        skip_feas = False
        if time_ is not None:
            firstcall = tkey not in time_dict
            if not firstcall:
                skip_feas = time_.value - time_dict[tkey] > POSTPONE

        if (time_ is None # not timed domain (do feas-motion)
                or firstcall # first call in timed domain (do feas-motion)
                or skip_feas): # postpone time passed (do motion only)
            with GlobalTimer.instance().block("holding_motion_gen"):
                assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
                conf1.assign()
                path, succ, feas = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                               conf1=conf1, conf2=conf2, grasp=grasp, fluents=fluents, tool=tool,
                                               tool_link=tool_link, base_link=base_link, timeout=timeout,
                                               show_state=show_state, approach_vec=approach_vec,
                                               skip_feas=skip_feas)

                if succ or skip_feas:
                    returneds.add(tkey) # success returned on first call or postpone time passed - mark finished case
                if not succ:
                    if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                    return None
                command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
                return (command,)
        else: # non-first call, postpone time not passed (X)
            return None
    return fn


def get_free_motion_gen_ori(robot, fixed=[], teleport=False, self_collisions=True, POSTPONE=POSTPONE_DEFAULT):
    def fn(conf1, conf2, time_=None, fluents=[]):
        with GlobalTimer.instance().block("free_motion_gen_ori"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            if teleport:
                path = [conf1.configuration, conf2.configuration]
            else:
                conf1.assign()
                obstacles = fixed + assign_fluent_state(fluents)
                path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=obstacles, self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                    return None
            command = Command([BodyPath(robot, path, joints=conf2.joints)])
            return (command,)
    return fn


def get_holding_motion_gen_ori(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, body, grasp, time_=None, fluents=[]):
        with GlobalTimer.instance().block("holding_motion_gen_ori"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            if teleport:
                path = [conf1.configuration, conf2.configuration]
            else:
                conf1.assign()
                obstacles = fixed + assign_fluent_state(fluents)
                attachment = grasp.attachment()
                attachment.assign()
                path = plan_joint_motion(robot, conf2.joints, conf2.configuration,
                                         obstacles=obstacles, attachments=[attachment], self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                    return None
            command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
            return (command,)
    return fn

##
# @brief check feasibility for given actor's grasping action to the body in pose with approach defined in grasp
# @return True if feasible.
def check_feas(pscene, body_subject_map, actor, checkers, home_dict, body, pose, grasp,
               base_link="base_link", show_state=False, no_approach=False):
    assert body == grasp.body
    gtimer = GlobalTimer.instance()
    if show_state:
        pscene.gscene.clear_highlight()
    with gtimer.block('check_feas'):
        subject = body_subject_map[body]
        Tbo = T_xyzquat(pose.value)
        subject.set_state(binding=(subject.oname, None, None, None),
                          state_param=(base_link, Tbo))

        Tlao = T_xyzquat(grasp.grasp_pose)
        Tboal = np.matmul(Tbo, SE3_inv(Tlao))
        if no_approach:
            Tboal_list = [Tboal]
        else:
            Tboal_ap = np.matmul(Tboal, T_xyzquat(grasp.approach_pose))
            Tboal_list = [Tboal, Tboal_ap]
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal)))
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal_ap)))

        ignore = []
        for k,v in body_subject_map.items():
            if k != body:
                ignore += [pscene.gscene.NAME_DICT[gname] for gname in v.geometry.get_family()]
        if show_state:
            display_bak = []
            for ig_tem in ignore:
                display_bak.append(ig_tem.display)
                ig_tem.display = False
                pscene.gscene.update_marker(ig_tem)

        if show_state:
            pscene.gscene.update_markers_all()
            pscene.gscene.add_highlight_axis("feas", "obj", "base_link",
                                             center=Tbo[:3,3], orientation_mat=Tbo[:3,:3])

        res = run_checkers(checkers, actor, subject, Tboal_list, home_dict, ignore, show_state=show_state)

        if show_state:
            for ig_tem, disp in zip(ignore, display_bak):
                ig_tem.display = disp
                pscene.gscene.update_marker(ig_tem)
    return res

##
# @brief run checkers
# @return True if feasible.
def run_checkers(checkers, actor, subject, Tloal_list, Q_dict, ignore=[], show_state=False, mplan=None):
    run_checkers.reason = None
    res = True
    gtimer = GlobalTimer.instance()
    btf_list = [BindingTransform(subject, None, actor, T_loal=Tloal) for Tloal in Tloal_list]
    for i_c, checker in enumerate(checkers):
        fname = checker.__class__.__name__
        run_checkers.reason = fname
        with gtimer.block(fname):
            for btf in btf_list:
                if not checker.check(actor, subject, None, btf, Q_dict, ignore=ignore):
                    res = False
                    break
        if mplan is not None and mplan.flag_log:
            mplan.result_log[fname].append(res)
        if not res:
            break
    if res:
        run_checkers.reason = "Success"

    if show_state:
        if subject.geometry.link_name == "base_link":
            Tbal_list = [Tloal for Tloal in Tloal_list]
        elif actor.geometry.link_name == "base_link":
            Tbal_list = [SE3_inv(Tloal) for Tloal in Tloal_list]
        else:
            raise(NotImplementedError("undefined binding case - possibly dual arm manipulation"))

        gscene = subject.geometry.gscene
        for i_t, Tbal in enumerate(Tbal_list):
            gscene.add_highlight_axis("feas", "tlink_{}".format(i_t), "base_link",
                                             center=Tbal[:3,3], orientation_mat=Tbal[:3,:3])

        if not res:
            print("Check Feas Fail: {}".format(checker.__class__.__name__))
            vis_bak = gscene.highlight_robot(color=gscene.COLOR_LIST[i_c])
        time.sleep(SHOW_TIME)
        if not res:
            gscene.recover_robot(vis_bak)
    return res

def reset_checker_cache():
    run_checkers.cache = {}

def get_checker_keys(tool, subject, fluents, conf1, body_subject_map, grasp=None):
    sname = subject.oname if subject is not None else None
    return (
        tool.name if tool is not None else None,
        sname,
        tuple(sorted([(fluent[0], fluent[1], fluent[2].index) for fluent in fluents
                      if body_subject_map[fluent[1]].oname != sname])),
        conf1.index, None if grasp is None else grasp.index)

class IK_Reason(Enum):
    success = 0
    ik_approach = 1
    col_approach = 2
    ik_grasp = 3
    col_grasp = 4
    approach_motion = 5

def get_ik_fn_rnb(pscene, body_subject_map, actor, checkers, home_dict, base_link="base_link", show_state=False,
                  disabled_collisions=set(), robot=0L, fixed=[], teleport=False, num_attempts=10, mplan=None,
                  timeout_single=0.01, POSTPONE=POSTPONE_DEFAULT):
    robot_name = [k for k, v in pscene.robot_chain_dict.items() if v["tip_link"]==actor.geometry.link_name][0]
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    home_pose = dict2list(home_dict, pscene.gscene.joint_names)
    time_dict = {}
    returneds = set()
    if GLOBAL_LOG_ENABLED:
        glog = GlobalLogger.instance()
        glog["ik_feas"] = []
        glog["ik_res"] = []
        glog["ik_feas_reason"] = []
        glog["ik_res_reason"] = []
        glog["ik_fail_args"] = []
    def fn(body, pose, grasp, time_=None):
        with GlobalTimer.instance().block("ik_fn"):
            tkey = (body, pose.index, grasp.index)
            if tkey in returneds:
                return None

            if show_state:
                pscene.gscene.show_pose(dict2list(home_dict, pscene.gscene.joint_names))

            no_approach = np.sum(np.abs(eye4 - T_xyzquat(grasp.approach_pose))) < 1e-6
            if time_ is None or tkey not in time_dict:
                feas = check_feas(pscene, body_subject_map, actor, checkers,
                                  home_dict, body, pose, grasp, base_link=base_link, show_state=show_state,
                                  no_approach=no_approach)

                if not feas and not DEBUG_MODE_PRIM_RNB:
                    fn.checkout_count += 1
                    time_dict[tkey] = time.time()
                    return None

                if GLOBAL_LOG_ENABLED:
                    glog['ik_feas'].append(feas)
                    glog['ik_feas_reason'].append(run_checkers.reason)
            elif time_.value-time_dict[tkey] < POSTPONE:
                return None
            returneds.add(tkey)

            obstacles = [body] + fixed
            set_pose(body, pose.pose)
            gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
            approach_pose = approach_from_grasp_tool_side(gripper_pose, grasp.approach_pose)
            # print("gripper_pose: {}".format(gripper_pose))
            # print("approach_pose: {}".format(approach_pose))
            ik_res_reason = IK_Reason.success
            with GlobalTimer.instance().block("ik_loop"):
                for i_ in range(num_attempts):
                    if show_state:
                        pscene.gscene.show_pose(dict2list(home_dict, pscene.gscene.joint_names))
                    with GlobalTimer.instance().block("ik_approch1"):
                        set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
                        # TODO: multiple attempts?
                        # GlobalLogger.instance()["ik_params"] = (robot, grasp.link, approach_pose)
                        if mplan is None:
                            q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
                        else:
                            q_approach = mplan.planner.solve_ik_py(robot_name, approach_pose[0]+approach_pose[1],
                                                                   timeout_single=timeout_single,
                                                                   self_collision=True, fulll_collision=False
                                                                   )
                            if q_approach is not None:
                                q_approach_dict = list2dict(q_approach, mplan.chain_dict[robot_name]["joint_names"])
                                home_dict_tmp = deepcopy(home_dict)
                                home_dict_tmp.update(q_approach_dict)
                                q_approach = tuple(dict2list(home_dict_tmp, pscene.gscene.joint_names))
                                set_joint_positions(robot, movable_joints, q_approach)
                    if show_state and q_approach is not None:
                        pscene.gscene.show_pose(q_approach)

                    if q_approach is not None:
                        with GlobalTimer.instance().block("pairwise_collision_a"):
                            pw_col = any(pairwise_collision(robot, b) for b in obstacles)
                    else:
                        pw_col = False

                    if (q_approach is None) or pw_col:
                        # print("obstacles: {}".format(obstacles))
                        if GLOBAL_LOG_ENABLED:
                            ik_res_reason = max(ik_res_reason,
                                                IK_Reason.ik_approach if q_approach is None else IK_Reason.col_approach,
                                                key=lambda x: x.value)
                        if show_state:
                            if q_approach is None:
                                print("inverse_kinematics fail to approach")
                                color = pscene.gscene.COLOR_LIST[2]
                            else:
                                print("IK-approach collision fail")
                                color = pscene.gscene.COLOR_LIST[0]
                            vis_bak = pscene.gscene.highlight_robot(color)
                            time.sleep(SHOW_TIME)
                            pscene.gscene.recover_robot(vis_bak)
                        continue
                    # print("go on")
                    conf = BodyConf(robot, q_approach)
                    if no_approach:
                        q_grasp = deepcopy(q_approach)
                    else:
                        with GlobalTimer.instance().block("ik_grasp1"):
                            if mplan is None:
                                q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
                            else:
                                q_grasp = mplan.planner.solve_ik_py(robot_name, gripper_pose[0]+gripper_pose[1],
                                                                    timeout_single=timeout_single,
                                                                    self_collision=True, fulll_collision=False
                                                                    )
                                q_grasp_dict = list2dict(q_grasp, mplan.chain_dict[robot_name]["joint_names"])
                                home_dict_tmp = deepcopy(home_dict)
                                home_dict_tmp.update(q_grasp_dict)
                                q_grasp = tuple(dict2list(home_dict_tmp, pscene.gscene.joint_names))
                                set_joint_positions(robot, movable_joints, q_grasp)
                        if show_state and q_grasp is not None:
                            pscene.gscene.show_pose(q_grasp)

                        if q_grasp is not None:
                            with GlobalTimer.instance().block("pairwise_collision_g"):
                                pw_col = any(pairwise_collision(robot, b) for b in obstacles)
                        else:
                            pw_col = False

                        if (q_grasp is None) or pw_col:
                            # print("obstacles: {}".format(obstacles))
                            if GLOBAL_LOG_ENABLED:
                                ik_res_reason = max(ik_res_reason,
                                                    IK_Reason.ik_grasp if q_grasp is None else IK_Reason.col_grasp,
                                                key=lambda x: x.value)
                            if show_state:
                                if q_grasp is None:
                                    print("inverse_kinematics fail to grasp")
                                    color = pscene.gscene.COLOR_LIST[2]
                                else:
                                    print("IK-grasp collision fail")
                                    color = pscene.gscene.COLOR_LIST[0]
                                vis_bak = pscene.gscene.highlight_robot(color)
                                time.sleep(SHOW_TIME)
                                pscene.gscene.recover_robot(vis_bak)
                            continue
                    if show_state:
                        time.sleep(SHOW_TIME)
                    if teleport or (no_approach and mplan is not None): # mplan does self-collision check
                        path = [q_approach, q_grasp]
                    else:
                        conf.assign()
                        #direction, _ = grasp.approach_pose
                        #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                        #                                   quat_from_pose(approach_pose))
                        with GlobalTimer.instance().block("plan_direct_joint_motion"):
                            path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles,
                                                            disabled_collisions=disabled_collisions)
                        if path is None:
                            print("Approach motion failed")
                            if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                            if GLOBAL_LOG_ENABLED:
                                ik_res_reason = max(ik_res_reason, IK_Reason.approach_motion,
                                                key=lambda x: x.value)
                            continue
                    command = Command([BodyPath(robot, path),
                                       Attach(body, robot, grasp.link),
                                       BodyPath(robot, path[::-1], attachments=[grasp])])
                    fn.pass_count += 1
                    if GLOBAL_LOG_ENABLED:
                        ik_res_reason = IK_Reason.success
                        glog['ik_res'].append(True)
                        glog['ik_res_reason'].append(ik_res_reason.name)
                    return (conf, command)
                    # TODO: holding collisions
                fn.fail_count += 1
                if GLOBAL_LOG_ENABLED:
                    glog['ik_res'].append(False)
                    glog['ik_res_reason'].append(ik_res_reason.name)
                    glog["ik_fail_args"].append((robot_name, approach_pose[0]+approach_pose[1],
                                                timeout_single, True, False))
                return None
    return fn

## @brief same as the original
def get_cfree_pose_pose_test_rnb(collisions=True, **kwargs):
    def test(b1, p1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p1.assign()
        p2.assign()
        res = not pairwise_collision(b1, b2, **kwargs)  # , max_distance=0.001)
        return res
    return test

## @brief fixed version - the original version transformed object to tool cooridnate
def get_cfree_obj_approach_pose_test_rnb(collisions=True):
    def test(b1, p1, g1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p2.assign()
        grasp_pose = p1.value
        approach_pose = multiply(p1.value, invert(g1.value), g1.approach, g1.value)
        for obj_pose in interpolate_poses(grasp_pose, approach_pose):
            set_pose(b1, obj_pose)
            if pairwise_collision(b1, b2):
                return False
        return True
    return test