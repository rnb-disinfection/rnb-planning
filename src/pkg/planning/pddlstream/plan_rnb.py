#!/usr/bin/env python

from __future__ import print_function

import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])

from pddlstream.algorithms.meta import solve, create_parser
from primitives_pybullet import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_stable_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen, get_movable_collision_test
from plan_pybullet import *
from primitives_rnb import *
from convert_pscene import *

from examples.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, get_pose, set_pose, Pose, \
    Point, stable_z, \
    BLOCK_URDF, SMALL_BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, HideOutput, wait_for_user, KUKA_IIWA_URDF, \
    LockRenderer, has_gui, draw_pose, is_darwin, disable_preview, CLIENTS,CLIENT, p, set_configuration
from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen, from_test, universe_test
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.algorithms.common import SolutionStore
from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    move_cost_fn, get_cfree_obj_approach_pose_test

#######################################################

def pddlstream_from_problem_rnb(pscene, robot, body_names, Q_init, goal_pairs=[], movable=[], checkers_ik=[],
                                tool_name=None, tool_link_name=None, mplan=None, timeout=TIMEOUT_MOTION_DEFAULT, teleport=False,
                                grasp_sample=SAMPLE_GRASP_COUNT_DEFAULT, stable_sample=SAMPLE_STABLE_COUNT_DEFAULT,
                                show_state=False, USE_MOVEIT_IK=False, TIMED_COMPLETE=False):
    print("================ MAKE PROBLEM ======================")
    print("IK checkers: {}".format([checker.__class__.__name__ for checker in checkers_ik]))
    print("MP checkers: {}".format([checker.__class__.__name__ for checker in mplan.motion_filters]))
    print("timeout motion : {}".format(timeout))
    print("====================================================")
    #assert (not are_colliding(tree, kin_cache))
    assert tool_link_name is not None, "tool_link_name should be passed to pddlstream_from_problem"
    assert tool_name is not None, "tool_name should be passed to pddlstream_from_problem"
    assert mplan is not None, "mplan should be passed to pddlstream_from_problem"

    if TIMED_COMPLETE:
        domain_pddl = read(get_file_path(__file__, 'domain/domain_timed.pddl'))
        stream_pddl = read(get_file_path(__file__, 'domain/stream_timed.pddl'))
    else:
        domain_pddl = read(get_file_path(__file__, 'domain/domain.pddl'))
        stream_pddl = read(get_file_path(__file__, 'domain/stream.pddl'))
    constant_map = {}

    print('Robot:', robot)
    set_configuration(robot, Q_init)
    conf = BodyConf(robot, get_configuration(robot))
    init = [('CanMove',),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]

    fixed = get_fixed(robot, movable)
    print('Movable:', movable)
    print('Fixed:', fixed)
    for body in movable:
        pose = BodyPose(body, get_pose(body))
        init += [('Timer', 0L),
                 ('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose)]
        for surface in fixed:
            init += [('Stackable', body, surface)]
            print("body {} - surface {}".format(body, surface))
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    body_subject_map = make_body_subject_map(pscene, body_names)
    body_actor_map = make_body_actor_map(pscene, body_names)
    subject_body_map = {sj.oname: bid for bid, sj in body_subject_map.items()}
    actor_body_map = {at.name: bid for bid, at in body_actor_map.items()}

    goal = ('and', ('AtConf', conf)) \
           + tuple([
               ('On', subject_body_map[sname], actor_body_map[aname]) for sname, aname in goal_pairs
           ])
            # ('Holding', body),
            # ('On', body, fixed[2]),
            # ('Cleaned', body),
            #             ('Cooked', body),
    APPROACH_VEC = 0.00 * Point(z=-1)
    actor = pscene.actor_dict[tool_name]
    # update_grasp_info({tool_name: GraspInfo(
    #     lambda body: sample_grasps(body_subject_map=body_subject_map, body=body, actor=actor,
    #                                sample_count=grasp_sample, show_state=show_state),
    #     approach_pose=Pose(APPROACH_VEC))})
    if USE_MOVEIT_IK:
        ik_kwargs = dict(mplan=mplan, timeout_single=0.01)
    else:
        ik_kwargs = {}
    ik_fun = get_ik_fn_rnb(
        pscene, body_subject_map, pscene.actor_dict[tool_name], checkers_ik, pscene.combined_robot.home_dict,
        disabled_collisions=get_disabled_collisions(pscene.gscene, robot),
        robot=robot, fixed=fixed, teleport=teleport, show_state=show_state, **ik_kwargs)

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen_rnb(body_subject_map, body_actor_map,
                                                      pscene.combined_robot.home_dict, fixed, show_state=show_state,
                                                      sample_count=stable_sample)),
        'sample-grasp': from_gen_fn(get_grasp_gen_rnb(body_subject_map, robot, tool_link_name, actor,
                                                      sample_count=grasp_sample, show_state=show_state,
                                                      approach_pose=Pose(APPROACH_VEC))),
        'inverse-kinematics': from_fn(ik_fun),

        'plan-free-motion': from_fn(get_free_motion_gen_rnb(mplan, body_subject_map, robot,
                                                            tool=pscene.actor_dict[tool_name],
                                                            tool_link=tool_link_name, timeout=timeout,
                                                            show_state=show_state,
                                                            approach_vec=APPROACH_VEC)),
        'plan-holding-motion': from_fn(
            get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool=pscene.actor_dict[tool_name],
                                       tool_link=tool_link_name, timeout=timeout, show_state=show_state,
                                       approach_vec=APPROACH_VEC)),
        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test_rnb()),
        'test-cfree-approach-pose': from_test(get_cfree_obj_approach_pose_test_rnb()),
        'test-cfree-traj-pose': from_test(negate_test(get_movable_collision_test())),  # get_cfree_traj_pose_test()),

        'TrajCollision': get_movable_collision_test()
    }

    if TIMED_COMPLETE:
        stream_map.update({'stream-time': from_gen_fn(get_time_gen())})

    reset_checker_cache()
    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal), ik_fun


def postprocess_plan(plan):
    paths = []
    for name, args in plan:
        if name == 'place':
            paths += args[-1].reverse().body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            paths += args[-1].body_paths
    return Command(paths)


def solve_in_pddlstream(pscene, mplan, ROBOT_NAME, TOOL_NAME, HOME_POSE, goal_pairs,
                        TIMEOUT_MOTION, MAX_TIME, MAX_ITER, MAX_SKELETONS,
                        GRASP_SAMPLE, STABLE_SAMPLE, SHOW_STATE, SEARCH_SAMPLE_RATIO,
                        use_pybullet_gui=False, USE_MOVEIT_IK=False, TIMED_COMPLETE=False, VERBOSE=False):
    gtimer = GlobalTimer.instance()
    gscene = pscene.gscene
#     checkers_ik = [checker for checker in mplan.motion_filters if checker.BEFORE_IK]
    checkers_ik = [checker for checker in mplan.motion_filters]
    mplan.motion_filters = [checker for checker in mplan.motion_filters if not checker.BEFORE_IK]
    checkers_ik_names = [checker.__class__.__name__ for checker in checkers_ik]
    checkers_mp_names = [checker.__class__.__name__ for checker in mplan.motion_filters]
    connect_notebook(use_gui=use_pybullet_gui)
    urdf_pybullet_path = copy_meshes(gscene)
    reset_pybullet()
    robot_body, body_names, movable_bodies = pscene_to_pybullet(
        pscene, urdf_pybullet_path, tool_name=TOOL_NAME, name_exclude_list=[ROBOT_NAME])
    print('Objects:', body_names)
    saver = WorldSaver()
    mplan.reset_log(True)
    problem, ik_fun = pddlstream_from_problem_rnb(pscene, robot_body, body_names=body_names,
                                                  Q_init=HOME_POSE,
                                                  goal_pairs=goal_pairs,
                                                  movable=movable_bodies,
                                                  checkers_ik=checkers_ik,
                                                  tool_name=TOOL_NAME,
                                                  tool_link_name=gscene.NAME_DICT[TOOL_NAME].link_name,
                                                  mplan=mplan, timeout=TIMEOUT_MOTION,
                                                  grasp_sample=GRASP_SAMPLE, stable_sample=STABLE_SAMPLE,
                                                  show_state=SHOW_STATE, USE_MOVEIT_IK=USE_MOVEIT_IK,
                                                  TIMED_COMPLETE=TIMED_COMPLETE)
    _, _, _, stream_map, init, goal = problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', str_from_object(set(stream_map)))
    ik_fun.checkout_count = 0
    ik_fun.fail_count = 0
    ik_fun.pass_count = 0
    with Profiler():
        with LockRenderer(lock=not True):
            gtimer.tic("plan")
            solution = solve(problem, algorithm='adaptive',
                             unit_costs=False, success_cost=INF, max_time=MAX_TIME, max_iterations=MAX_ITER,
                             max_skeletons=MAX_SKELETONS, search_sample_ratio=SEARCH_SAMPLE_RATIO, verbose=VERBOSE)
            gtimer.toc("plan") / 1000
            saver.restore()
    print_solution(solution)
    plan, cost, evaluations = solution
    res = not any(plan is status for status in [None, False])

    move_num = len(plan) if res else 0
    pre_motion_checks = mplan.result_log["pre_motion_checks"]
    plan_try = len(pre_motion_checks)
    planning_log = mplan.result_log["planning"]
    plan_num = len(planning_log)
    fail_num = np.sum(np.logical_not(mplan.result_log["planning"]))
    elapsed = SolutionStore.last_log['run_time']

    log_dict = {"plan_time": elapsed, "length": move_num,
                "IK_tot": ik_fun.checkout_count + ik_fun.pass_count + ik_fun.fail_count,
                "IK_count": ik_fun.pass_count + ik_fun.fail_count, "failed_IKs": ik_fun.fail_count,
                "MP_tot": plan_try, "MP_count": plan_num, "failed_MPs": fail_num,
                "success": res, "body_names": body_names, "plan": plan,
                "pre_motion_checks": pre_motion_checks, "planning_log": planning_log,
                "checkers_ik": checkers_ik_names, "checkers_mp":checkers_mp_names,
                "time_log": gtimer.timelist_dict if gtimer.stack else gtimer.time_dict}
    return res, plan, log_dict
