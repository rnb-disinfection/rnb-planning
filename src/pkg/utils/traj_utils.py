import numpy as np
from .utils import sign_positive_bias, list2dict
from .rotation_utils import *
from copy import deepcopy

##
# @brief calculate cubic interpolation coefficients (x_t = a*t^3+b*t^2+c*t+d)
# @param dt  step time between q0, q1, q2
# @param v0 initial velocity at q0
# @return a,b,c,d in (x_t = a*t^3+b*t^2+c*t+d)
def calc_cubic_coeffs(dt, v0, q0, q1, q2):
    Tmat = np.array([[dt ** 3, dt ** 2], [8 * dt ** 3, 4 * dt ** 2]])  # (2,2)
    VQ = np.array([q1 - v0 * dt - q0, q2 - 2 * v0 * dt - q0])  # (2,DIM)
    if len(VQ.shape) == 1:
        VQ = VQ[:, np.newaxis]
    # Tmat*[a,b]'=VQ
    a, b = np.matmul(np.linalg.inv(Tmat), VQ)
    return a, b, v0, q0


##
# @brief calculate cubic interpolated trajectory (x_t = a*t^3+b*t^2+c*t+d)
def calc_cubic_traj(t, a, b, c, d):
    return a * t ** 3 + b * t ** 2 + c * t + d


##
# @brief calculate cubic interpolated trajectory velocity (v_t = 3*a*t^2+2*b*t+c)
def calc_cubic_vel(t, a, b, c):
    return 3 * a * t ** 2 + 2 * b * t + c


##
# @brief waypoint period to observe acc&vel limits with cubic interpolation
# @remark   Check velocity and acceleration limits by |3at^2+2bt+c|<Vmax, |6at+2b|<Amax. \n
#           Applying time scale r to at^3+bt^2+ct+d=Q makes ar^3t^3+br^2t^2+crt+d=Q,
#           makeing a_new = ar^3, b_new=br^2, c_new = cr. \n
#           But this implementation is only an estimation and does not give an optimal answer,
#           as c_new = c = v0 is a given constant value.
# @dt_step minimal timestep
# @return dt, a, b, c, d
def calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, vel_lim, acc_lim, dt_init=None):
    all_pass = False
    dt_cur = np.max(np.abs(q1 - q0) / vel_lim) if dt_init is None else dt_init
    if dt_cur < dt_step:
        dt_cur = dt_step

    while not all_pass:
        a, b, c, d = calc_cubic_coeffs(dt_cur, v0, q0, q1, q2)
        a6T, b2 = 6 * a * dt_cur, 2 * b
        a6T_b2 = a6T + b2
        a3T2, b2T = 3 * a * dt_cur ** 2, 2 * b * dt_cur
        a3T2_b2T_c = a3T2 + b2T + c

        if np.max(np.abs(b2) / acc_lim) > 1:                # at t=0, |2b|<Amax
            ratio = np.sqrt(np.min(acc_lim/np.abs(b2))) * 0.99  # give 1% margin
        elif np.max(np.abs(a6T_b2) / acc_lim) > 1:            # at t=T, |6aT+2b|<Amax
            dir_c2 = sign_positive_bias(a6T_b2)
            roots = [np.roots([c1, c2, 0, c4]) for c1, c2, c4 in zip(a6T, b2,  -dir_c2*acc_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99          # give 1% margin
        elif np.max(np.abs(a3T2_b2T_c) / vel_lim) > 1:            # at t=T, |3aT^2+2bT+c|<Vmax
            dir_c3 = sign_positive_bias(a3T2_b2T_c)
            roots = [np.roots([c1, c2, c3, c4]) for c1, c2, c3, c4 in zip(a3T2, b2T, c,  -dir_c3*vel_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99          # give 1% margin
        else:
            all_pass = True
            continue

        dt_cur = dt_cur/ratio
        dt_cur = np.ceil(dt_cur / dt_step) * dt_step
    return dt_cur, a, b, c, d


##
# @brief get safe cubic interpolated trajectories
def get_safe_cubics(dt_step, trajectory, vel_lim, acc_lim, slow_start=False):
    T_list = []
    Q_list = []
    V_list = []
    v0 = np.zeros_like(trajectory[0])
    traj_len = len(trajectory)
    for i in range(traj_len):
        q0 = trajectory[i]
        q1 = trajectory[i + 1] if i + 1 < traj_len else trajectory[-1]
        q2 = trajectory[i + 2] if i + 2 < traj_len else trajectory[-1]
        if slow_start:
            dt, a, b, c, d = calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, # first 1/4 is slow-start
                                                    vel_lim=vel_lim*max(0.2, min(1, (float(i+1)/traj_len)**2*4)),
                                                    acc_lim=acc_lim*max(0.2, min(1, (float(i+1)/traj_len)**2*4)))
        else:
            dt, a, b, c, d = calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, vel_lim=vel_lim, acc_lim=acc_lim)
        v0 = calc_cubic_vel(dt, a, b, c)
        T_list.append(dt)
        Q_list.append(q0)
        V_list.append(v0)
    return T_list, Q_list, V_list


##
# @brief get full cubic trajectory for given waypoints
# @param dt_step    control time step
# @param T_list     List of time for each waypoint to the next one, last has no meaning-give 0. \n
#                   Each T in T list should be integer multiplication of dt_step
# @param Q_list     List of waypoint values (vectors)
def get_traj_all(dt_step, T_list, Q_list):
    t_all = []
    traj_all = []
    traj_len = len(Q_list)
    t0 = 0
    v0 = np.zeros_like(Q_list[0])
    for i in range(traj_len):
        T = T_list[i]
        if T <= dt_step:
            continue
        q0 = Q_list[i]
        q1 = Q_list[i + 1] if i + 1 < traj_len else Q_list[-1]
        q2 = Q_list[i + 2] if i + 2 < traj_len else Q_list[-1]
        a, b, c, d = calc_cubic_coeffs(T, v0, q0, q1, q2)
        for t in np.arange(0, T-dt_step/2, dt_step):
            t_all.append(t0 + t)
            traj_all.append(calc_cubic_traj(t, a, b, c, d))
        v0 = calc_cubic_vel(T, a, b, c)
        t0 += T
    return t_all, traj_all


##
# @brief get full cubic trajectory for given waypoint trajectory
# @remark terminal deceleration considered
def calc_safe_cubic_traj(dt_step, trajectory, vel_lim, acc_lim, slow_start=True):
    print("WARNING: This function is deprecated. USE calc_safe_trajectory")
    # calculate trajectory in forward and backward direction to consider deceleration
    T_list, Q_list, _ = get_safe_cubics(dt_step, trajectory, vel_lim=vel_lim, acc_lim=acc_lim, slow_start=slow_start)
    Trev_list, Qrev_list, _ = get_safe_cubics(dt_step, np.array(list(reversed(trajectory))), vel_lim=vel_lim,
                                              acc_lim=acc_lim, slow_start=slow_start)

    # weighting values to mix waypoint times, in S curves
    Tlen = len(T_list)-1
    alphaT = (np.arange(Tlen).astype(np.float) / (Tlen - 1))
    alphaT = (np.cos((alphaT + 1) * np.pi) + 1) / 2     # backward weights
    betaT = 1 - alphaT                                  # forward weights
    # mix waypoint times with weights
    T_list = list(((betaT*np.array(T_list[:-1])
                    + alphaT*np.array(list(reversed(Trev_list[:-1]))))
                   / dt_step).astype(np.int)*dt_step) # make sure each T is integer multiplication of dt_step
    T_list, Trev_list = T_list + [0], list(reversed(T_list)) + [0]
    # T_accum = [np.sum(T_list[:i]) for i in range(len(T_list))]

    # re-calculate trajectory with mixed waypoint times
    t_all, traj_all = get_traj_all(dt_step, T_list, Q_list)
    trev_all, trajrev_all = get_traj_all(dt_step, Trev_list, Qrev_list)
    traj_all, trajrev_all = np.array(traj_all), np.array(list(reversed(trajrev_all)))

    # weighting values to mix waypoint times, in S curves
    traj_len = len(traj_all)
    alpha = (np.arange(traj_len).astype(np.float) / (traj_len - 1))[:, np.newaxis]
    alpha = (np.cos((alpha + 1) * np.pi) + 1) / 2
    beta = 1 - alpha

    # mix forward and backward trajectory with S curve weights
    traj_tot = beta * traj_all + alpha * trajrev_all
    return traj_tot


##
# @brief calculate safe waypoint-time list for a list of waypoints, by simply iterating velocity & acceleration limits
# @param trajectory list of waypoints
# @param vel_lims   velocity limits, either in vector or scalar
# @param acc_lims   acceleration limits, either in vector or scalar
def calc_T_list_simple(trajectory, vel_lims, acc_lims,
                        vel_margin_ratio=0.1, acc_margin_ratio=0.1,
                        upper_bound_ratio=1.2, low_bound_ratio=0.5):
    dq_list = np.subtract(trajectory[1:], trajectory[:-1])
    idx_skip = np.where(np.linalg.norm(dq_list, axis=-1)<1e-3)[0]
    idx_valid = np.where(np.linalg.norm(dq_list, axis=-1)>=1e-3)[0]
    dq_list = dq_list[idx_valid]
    dT_list = np.max(np.abs(dq_list)/vel_lims, axis=-1)
    dT_list_mid_pre = None
    alpha = 0.5
    for _ in range(1000):
        dT_list_pre = dT_list # between q, N-1
        vel_list = dq_list/dT_list[:,np.newaxis] # between q, N-1
        vel_list = np.pad(vel_list, ((1,1),(0,0)), 'constant', constant_values=0) # between q and expand, N+1
        dT_list = np.pad(dT_list, (1,1), 'constant', constant_values=0) # between q and expand, N+1
        dv_list = vel_list[1:]- vel_list[:-1] # on q, N
        vel_mid_list = np.abs((vel_list[1:] + vel_list[:-1])/2) # on q, N
        vel_ratio_mid = np.max(vel_mid_list/(vel_lims*(1+vel_margin_ratio)), axis=-1) # on q, N
        dT_list_mid = (dT_list[:-1]+dT_list[1:])/2 # on q, N
        acc_list = np.abs(dv_list) / dT_list_mid[:,np.newaxis] # on q, N
        acc_ratio = np.max(acc_list/(acc_lims*(1+acc_margin_ratio)), axis=-1) # on q, N
        vel_pass = vel_ratio_mid < upper_bound_ratio
        acc_pass = acc_ratio < upper_bound_ratio
        low_bound_pass = np.logical_or(vel_ratio_mid > low_bound_ratio, acc_ratio > low_bound_ratio)
        # calc_T_list_simple.pass_list= [vel_pass, acc_pass, low_bound_pass]
        # calc_T_list_simple.ratio_list= [vel_ratio_mid, acc_ratio]
        if  np.all(vel_pass) and np.all(acc_pass) and np.all(low_bound_pass):
            dT_list = dT_list[1:-1]
            break
        adjust_ratio = np.max([
                                acc_ratio*np.logical_not(acc_pass),
                                vel_ratio_mid*np.logical_not(vel_pass),
                                np.maximum(acc_ratio, vel_ratio_mid)*np.logical_not(low_bound_pass)], axis=0) \
                                +np.all([acc_pass, vel_pass, low_bound_pass], axis=0) # on q, N
        if dT_list_mid_pre is None:
            dT_list_mid_new = dT_list_mid*adjust_ratio # on q, N
        else:
            dT_list_mid_new = alpha*dT_list_mid*adjust_ratio + (1-alpha)*dT_list_mid_pre # on q, N
        dT_list_mid_pre = deepcopy(dT_list_mid_new)
        dT_list_mid_new[0] *= 2   # double first and last acc time as they are half-step
        dT_list_mid_new[-1] *= 2   # double first and last acc time as they are half-step
        dT_list = (dT_list_mid_new[:-1]+dT_list_mid_new[1:])/2 # between q, N-1
    T_list = np.pad(dT_list, (0,1), 'constant', constant_values=0)
    for idx in idx_skip:
        T_list = np.insert(T_list, idx, 0)
    return T_list


##
# @brief get full cubic trajectory for given waypoint trajectory
# @remark terminal deceleration considered
def calc_safe_trajectory(dt_step, trajectory, vel_lims, acc_lims):
    # calculate waypoint times
    T_list = calc_T_list_simple(trajectory, vel_lims, acc_lims)

    # round waypoint times with dt_step
    T_list_new = []
    Q_list = []
    T_stack = 0
    for Q, T in zip(trajectory, T_list):
        T_stack += T
        if T_stack > dt_step:
            T_list_new.append(np.ceil(T_stack / dt_step) * dt_step)
            Q_list.append(Q)
            T_stack = 0
    if np.any(Q != Q_list[-1]):
        T_list_new.append(np.ceil(T_stack / dt_step) * dt_step)
        Q_list.append(Q)
    T_list = np.array(T_list_new)
    Q_list = np.array(Q_list)

    # prepare backward waypoints and times
    Qrev_list = np.array(list(reversed(Q_list)))
    Trev_list = list(reversed(T_list[:-1])) + [0]

    # re-calculate trajectory with mixed waypoint times
    t_all, traj_all = get_traj_all(dt_step, T_list, Q_list)
    trev_all, trajrev_all = get_traj_all(dt_step, Trev_list, Qrev_list)
    traj_all, trajrev_all = np.array(traj_all), np.array(list(reversed(trajrev_all)))

    # weighting values to mix waypoint times, in S curves
    traj_len = len(traj_all)
    alpha = (np.arange(traj_len).astype(np.float) / (traj_len - 1))[:, np.newaxis]
    alpha = (np.cos((alpha + 1) * np.pi) + 1) / 2
    beta = 1 - alpha

    # mix forward and backward trajectory with S curve weights
    traj_tot = beta * traj_all + alpha * trajrev_all
    return t_all, traj_tot


##
# @brief calculate safe trajectory for all SearchNode in schedule
def calculate_safe_schedule(pscene, snode_schedule, vel_lims, acc_lims, dt_step=2e-2):
    snode_schedule_safe = []
    for snode in snode_schedule:
        snode_cp = snode.copy(pscene)
        snode_schedule_safe.append(snode_cp)
        if snode_cp.traj is not None:
            _, traj_new = calc_safe_trajectory(dt_step, snode_cp.traj, vel_lims=vel_lims, acc_lims=acc_lims)
            snode_cp.set_traj(traj_new)
    return snode_schedule_safe


def copy_schedule(pscene, snode_schedule):
    copied = []
    for snode in snode_schedule:
        copied.append(snode.copy(pscene))
    return copied

##
# @brief simplify trajectory - divide each constant velocity section into 5 waypoints
def simplify_traj(trajectory, step_fractions=[0, 0.1, 0.5, 0.9, 1]):
    dq_pre = np.zeros(trajectory.shape[-1])
    traj_new = []
    traj_stack = []
    step_len = len(step_fractions)
    for q, q_nxt in zip(trajectory, np.pad(trajectory[1:], ((0, 1), (0, 0)), 'edge')):
        dq = (q_nxt-q)
        dq = dq / (np.linalg.norm(dq)+1e-16)
        traj_stack.append(q)
        if np.max(np.abs(dq-dq_pre)) > 1e-5:
            if len(traj_stack) > 1:
                if len(traj_stack) > step_len:
                    traj_stack = traj_stack[0] \
                                 + (traj_stack[-1] - traj_stack[0]) * np.array(step_fractions)[:, np.newaxis]
                traj_new.append(traj_stack[:-1])
            traj_stack = [q]
        dq_pre = dq
    traj_new.append(traj_stack[-1:])
    return np.concatenate(traj_new)


##
# @brief simplify_schedule
def simplify_schedule(pscene, snode_schedule, step_fractions=[0, 0.1, 0.5, 0.9, 1]):
    snode_schedule_cp = copy_schedule(pscene, snode_schedule)
    for snode in snode_schedule_cp:
        if snode.traj is not None:
            snode.set_traj(simplify_traj(snode.traj, step_fractions=step_fractions))
    return snode_schedule_cp


##
# @brief   generate new schedule with mixed dual arm motion
# @param    mplan           rnb-planning.src.pkg.planning.motion.moveit.moveit_planner.MoveitPlanner
# @param    snode_schedule  list of SearchNode
def mix_schedule(mplan, snode_schedule):
    schedule_len = len(snode_schedule)
    mixed_prev = False
    snode_schedule_mixed = []
    snode_pre = snode_schedule[0]
    for i_s in range(schedule_len):
        snode_cur = snode_schedule[i_s]
        mplan.pscene.set_object_state(snode_pre.state)

        snode_new = snode_cur.copy(mplan.pscene)
        if mixed_prev or \
                snode_pre.traj is None or len(snode_pre.traj) == 0 \
                or snode_cur.traj is None or len(snode_cur.traj) == 0:
            mixed_prev = False
        else:
            traj_pre = snode_pre.traj
            traj_cur = snode_cur.traj
            stay_jidx_pre = np.where(traj_pre[0] == traj_pre[-1])[0]
            move_jidx_cur = np.where(traj_cur[0] != traj_cur[-1])[0]
            if all([ji in stay_jidx_pre for ji in move_jidx_cur]) \
                    and all([ji in move_jidx_cur for ji in stay_jidx_pre]):
                len_pre, len_cur = len(traj_pre), len(traj_cur)
                mix_ratio_list = [1., 3. / 4, 2. / 4, 1. / 4]
                for mix_ratio in mix_ratio_list:
                    mix_idx = max(0, int(len_pre - len_cur * mix_ratio))
                    mix_len = max(mix_idx + len_cur, len_pre)
                    traj_mix = np.zeros((mix_len, mplan.joint_num))
                    traj_mix[:len_pre] = traj_pre
                    traj_mix[len_pre:] = traj_pre[-1:]
                    traj_mix[mix_idx:, move_jidx_cur] = traj_cur[:, move_jidx_cur]
                    res = mplan.validate_trajectory(traj_mix)
                    if res:
                        snode_new.set_traj(traj_mix)
                        snode_new.parents = snode_pre.parents
                        snode_schedule_mixed.pop(-1)
                        mixed_prev = True
                        break
                    else:
                        mixed_prev = False
            else:
                mixed_prev = False
        snode_schedule_mixed.append(snode_new)
        snode_pre = snode_cur

    return snode_schedule_mixed


##
# @brief calculate sweep trajectory
# @param gtem GeometryItem
# @param dP_tar target delta pos
def get_sweep_traj(mplan, gtem, dP_tar, Q0, DP=0.01, ERROR_CUT=0.01, SINGULARITY_CUT = 0.01, VISUALIZE=False, VERBOSE=False):
    gscene = gtem.gscene
    joint_limits = [(gscene.urdf_content.joint_map[jname].limit.lower,
                     gscene.urdf_content.joint_map[jname].limit.upper) for jname in gscene.joint_names]
    Q=Q0
    T0 = Tnew = gtem.get_tf(list2dict(Q0, gscene.joint_names))
    P0 = T0[:3,3]
    if VISUALIZE:
        gscene.show_pose(Q)
    Traj = []
    dPnorm = np.linalg.norm(dP_tar)
    DIR = np.concatenate([np.divide(dP_tar, dPnorm), [0,0,0]])
    P0 = np.concatenate([P0, [0,0,0]])
    reason = "end"
    singularity = False
    for dP_cur in np.arange(0, dPnorm+DP/2, DP):
        Jac = gtem.get_jacobian(Q)
        if np.min(np.abs(np.real(np.linalg.svd(Jac)[1]))) <= SINGULARITY_CUT:
            singularity = True
            reason = "singular"
            break
        Jinv = np.linalg.inv(Jac)
        Ptar = np.multiply(DIR, dP_cur) + P0
        Pcur = np.concatenate([Tnew[:3,3], [0,0,0]])
        dQ=np.matmul(Jinv, Ptar-Pcur)
        Q = Q+dQ
        if VISUALIZE:
            gscene.show_pose(Q)
        dlim = np.subtract(joint_limits, Q[:, np.newaxis])
        if np.any(dlim[:,0] > 0):
            reason = "joint min"
            break
        if np.any(dlim[:,1] < 0):
            reason = "joint max"
            break
        if not mplan.validate_trajectory([Q]):
            reason = "collision"
            break
        Tnew = gtem.get_tf(list2dict(Q, gscene.joint_names))
        dPcalc = Tnew[:3, 3] - T0[:3, 3]
        dRcalc = Rotation.from_dcm(np.matmul(Tnew[:3, :3], T0[:3, :3].transpose())).as_rotvec()
        if np.abs(np.linalg.norm(dPcalc)- np.dot(dPcalc, DIR[:3]))>ERROR_CUT:
            reason = "error off"
            break
        Traj.append(Q)
    if VERBOSE:
        print(reason)
    return np.array(Traj), reason=="end"

