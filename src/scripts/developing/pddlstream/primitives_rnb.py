import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from pkg.utils.rotation_utils import *
from pkg.utils.utils import *
from primitives_pybullet import *

def plan_motion(mplan, body_subject_map, conf1, conf2, grasp, fluents, tool_link, base_link="base_link"):
    pscene = mplan.pscene
    for fluent in fluents:
        subject = body_subject_map[fluent[1]]
        Tbase = T_xyzquat(fluent[2].value)
        subject.geometry.set_link(base_link)
        subject.geometry.set_offset_tf(center=Tbase[:3,3], orientation_mat=Tbase[:3,:3])
    if grasp is not None:
        graspped = body_subject_map[grasp.body]
        Tgrasp = T_xyzquat(grasp.grasp_pose)
        graspped.geometry.set_link(tool_link)
        graspped.geometry.set_offset_tf(center=Tgrasp[:3,3], orientation_mat=Tgrasp[:3,:3])
    Qcur, Qto = conf1.values, conf2.values
    from_state = pscene.initialize_state(np.array(Qcur))
    to_state = from_state.copy(pscene)
    to_state.Q = np.array(Qto)
    Traj, LastQ, error, success, binding_list = mplan.plan_transition(from_state, to_state, {}, timeout=3)
    return Traj, success


def get_free_motion_gen_rnb(mplan, body_subject_map, robot, tool_link, base_link="base_link"):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        conf1.assign()
        path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                 conf1=conf1, conf2=conf2, grasp=None, fluents=fluents, tool_link=tool_link, base_link=base_link)
        if not succ:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn


def get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool_link, base_link="base_link"):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        conf1.assign()
        path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                 conf1=conf1, conf2=conf2, grasp=grasp, fluents=fluents, tool_link=tool_link, base_link=base_link)
        if not succ:
            if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
        return (command,)
    return fn