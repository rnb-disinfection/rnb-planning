from demo_config import *
from pkg.utils.rotation_utils import *
from pkg.planning.constraint.constraint_subject import SweepLineTask

def get_jacobian(gscene, gtem, Q):
    Q_dict = list2dict(Q, gscene.joint_names)
    Jac = []
    for ij, jname in enumerate(gscene.joint_names):
        joint = gscene.urdf_content.joint_map[jname]
        Tj = T_xyzrpy((joint.origin.xyz, joint.origin.rpy))
        T_link = get_tf(joint.parent, Q_dict, gscene.urdf_content)
        T_bj = np.matmul(T_link, Tj)
        zi = np.matmul(T_bj[:3, :3], joint.axis)
        T_p = gtem.get_tf(Q_dict)
        dpi = T_p[:3, 3] - T_bj[:3, 3]
        zp = np.cross(zi, dpi)
        Ji = np.concatenate([zp, zi])
        Jac.append(Ji)
    Jac = np.array(Jac).transpose()
    return Jac


def make_sweep_traj(gscene, mplan, gtem, Traj, len_traj=None):
    SINGULARITY_CUT = 0.01
    Qi = Traj[0]
    Qf = Traj[-1]
    if len_traj is None:
        len_traj = len(Traj)

    Qidict = list2dict(Qi, gscene.joint_names)
    Qfdict = list2dict(Qf, gscene.joint_names)
    Ti = gtem.get_tf(Qidict)
    Tf = gtem.get_tf(Qfdict)
    dPabs = np.linalg.norm(Tf[:3, 3] - Ti[:3, 3])
    DP = dPabs / len_traj
    DIR = np.concatenate([(Tf[:3, 3] - Ti[:3, 3]) / dPabs, [0] * 3])
    Q = Qi
    singularity = False
    Traj_wipe = [Qi]
    for _ in range(len_traj):
        Jac = get_jacobian(gscene, gtem, Q)
        if np.min(np.abs(np.real(np.linalg.svd(Jac)[1]))) <= SINGULARITY_CUT:
            singularity = True
            print("singular")
            break
        Jinv = np.linalg.inv(Jac)
        dQ = np.matmul(Jinv, np.multiply(DIR, DP))
        Q = Q + dQ
        Traj_wipe.append(Q)
        dlim = np.subtract(RobotSpecs.get_joint_limits(RobotType.indy7), Q[:, np.newaxis])
        if np.any(dlim[:, 0] > 0):
            print("min lim: {}".format(np.where(dlim[:, 0] > 0)[0]))
            break
        if np.any(dlim[:, 1] < 0):
            print("max lim: {}".format(np.where(dlim[:, 1] < 0)[0]))
            break
    #         if not mplan.validate_trajectory([Q]):
    #             print("col")
    #             break
    #         Tnew = gtem.get_tf(list2dict(Q, gscene.joint_names))
    #         if np.abs(Ti[0,3]-Tnew[0,3])>0.01:
    #             print("off")
    #             break
    Traj_wipe.append(Qf)
    Traj_wipe = np.array(Traj_wipe)
    return Traj_wipe


def refine_sweep(pscene, mplan, snode_schedule, len_traj=None):
    for snode_pre, snode in zip(snode_schedule[:-1], snode_schedule[1:]):
        breaker = False
        for sname, stype, ntem1, ntem2 in zip(pscene.subject_name_list, pscene.subject_type_list,
                                              snode_pre.state.node, snode.state.node):
            if ntem1 != ntem2 and stype == SweepLineTask:
                if ntem1 == 1:
                    gtem = gscene.NAME_DICT[snode.state.binding_state[sname].chain.actor_root_gname]
                    snode.traj = make_sweep_traj(pscene.gscene, mplan, gtem, snode.traj, len_traj=len_traj)


def simplify_sweep(pscene, mplan, snode_schedule, len_traj=None):
    for snode_pre, snode in zip(snode_schedule[:-1], snode_schedule[1:]):
        breaker = False
        for sname, stype, ntem1, ntem2 in zip(pscene.subject_name_list, pscene.subject_type_list,
                                              snode_pre.state.node, snode.state.node):
            if ntem1 != ntem2 and stype == SweepLineTask:
                if ntem1 == 1:
                    gtem = pscene.gscene.NAME_DICT[snode.state.binding_state[sname].chain.actor_root_gname]
                    step = int(len(snode.traj) / len_traj)
                    Qlast = snode.traj[-1]
                    snode.traj = snode.traj[::step]
                    snode.traj = np.array(
                        list(snode.traj) + [Qlast]
                    )
