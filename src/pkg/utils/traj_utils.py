import numpy as np
from .utils import sign_positive_bias

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
# @dt_step minimal timestep
# @return dt, a, b, c, d
def calc_cubic_coeffs_safe(dt_step, v0, q0, q1, q2, vel_lim, acc_lim, dt_init=None):
    all_pass = False
    dt_cur = np.max(np.abs(q1 - q0) / vel_lim) if dt_init is None else dt_init
    if dt_cur < dt_step:
        return 0, 0, 0, v0, q0
    case = 0
    while not all_pass:
        a, b, c, d = calc_cubic_coeffs(dt_cur, v0, q0, q1, q2)
        a6T, b2 = 6 * a * dt_cur, 2 * b
        a6T_b2 = a6T + b2
        a3T2, b2T_v0 = 3 * a * dt_cur ** 2, 2 * b * dt_cur + c
        a3T2_b2T_v0 = a3T2 + b2T_v0
        if case > 0:
            # if case == 1:
            #     print("res1 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(b2),2), np.round(acc_lim, 2)))
            if case == 2: # This is remained because case 2 is not tested yet
                print("res2 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a6T_b2),2), np.round(acc_lim,2)))
            # if case == 3:
            #     print("res3 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a3T2_b2T_v0), 2), np.round(vel_lim, 2)))

        if np.max(np.abs(b2)-acc_lim) > 0:
            # case = 1
            # print("case1 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(b2),2), np.round(acc_lim, 2)))
            ratio = np.sqrt(np.min(acc_lim/np.abs(b2))) * 0.99
        elif np.max(np.abs(a6T_b2)-acc_lim) > 0:
            case = 2 # This is remained because case 2 is not tested yet
            print("case2 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a6T_b2),2), np.round(acc_lim,2)))
            dir_c2 = sign_positive_bias(a6T_b2)
            roots = [np.roots([c1, c2, 0, c4]) for c1, c2, c4 in zip(a6T, b2,  -dir_c2*acc_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99  # give 1% margin
        elif np.max(np.abs(a3T2_b2T_v0)-vel_lim) > 0:
            # case = 3
            # print("case3 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a3T2_b2T_v0), 2), np.round(vel_lim, 2)))
            dir_c3 = sign_positive_bias(a3T2_b2T_v0)
            roots = [np.roots([c1, c2, 0, c4]) for c1, c2, c4 in zip(a3T2, b2T_v0,  -dir_c3*vel_lim)]
            ratio = np.min(
                [np.min([np.real(rt) for rt in root_single if np.imag(rt) < 1e-6 and np.real(rt) > 0]) for
                 root_single in roots]) * 0.99  # give 1% margin
        else:
            case = 0
            # print("res1 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(b2),2), np.round(acc_lim, 2)))
            # print("res2 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a6T_b2),2), np.round(acc_lim,2)))
            # print("res3 {}: \n{} \n{}".format(np.round(dt_cur, 2), np.round(np.abs(a3T2_b2T_v0), 2), np.round(vel_lim, 2)))
            all_pass = True
            continue

        dt_cur = dt_cur/ratio
        dt_cur = np.ceil(dt_cur / dt_step) * dt_step
    return dt_cur, a, b, c, d


##
# @brief get safe cubic interpolated trajectories
def get_safe_cubics(dt_step, trajectory, vel_lim, acc_lim):
    T_list = []
    Q_list = []
    V_list = []
    v0 = np.zeros_like(trajectory[0])
    traj_len = len(trajectory)
    for i in range(traj_len):
        q0 = trajectory[i]
        q1 = trajectory[i + 1] if i + 1 < traj_len else trajectory[-1]
        q2 = trajectory[i + 2] if i + 2 < traj_len else trajectory[-1]
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
def calc_safe_cubic_traj(dt_step, trajectory, vel_lim, acc_lim):
    T_list, Q_list, _ = get_safe_cubics(dt_step, trajectory, vel_lim=vel_lim, acc_lim=acc_lim)
    Trev_list, Qrev_list, _ = get_safe_cubics(dt_step, np.array(list(reversed(trajectory))), vel_lim=vel_lim,
                                              acc_lim=acc_lim)
    Tlen = len(T_list)-1
    alphaT = (np.arange(Tlen).astype(np.float) / (Tlen - 1))
    alphaT = (np.cos((alphaT + 1) * np.pi) + 1) / 2
    betaT = 1 - alphaT
    T_list = list(((alphaT*np.array(T_list[:-1])
                    + betaT*np.array(list(reversed(Trev_list[:-1]))))
                   / dt_step).astype(np.int)*dt_step) # make sure each T is integer multiplication of dt_step
    Trev_list = list(reversed(T_list)) + [0]
    T_list = T_list + [0]
    # T_accum = [np.sum(T_list[:i]) for i in range(len(T_list))]
    t_all, traj_all = get_traj_all(dt_step, T_list, Q_list)
    trev_all, trajrev_all = get_traj_all(dt_step, Trev_list, Qrev_list)
    traj_all = np.array(traj_all)
    trajrev_all = np.array(list(reversed(trajrev_all)))
    traj_len = len(traj_all)
    alpha = (np.arange(traj_len).astype(np.float) / (traj_len - 1))[:, np.newaxis]
    alpha = (np.cos((alpha + 1) * np.pi) + 1) / 2
    beta = 1 - alpha
    traj_tot = beta * traj_all + alpha * trajrev_all
    return traj_tot