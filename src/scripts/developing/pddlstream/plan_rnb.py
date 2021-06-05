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
from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    move_cost_fn, get_cfree_obj_approach_pose_test

#######################################################

def pddlstream_from_problem_rnb(pscene, robot, body_names, movable=[], checkers=[],
                                tool_name=None, tool_link_name=None, mplan=None, teleport=False,
                                grasp_sample=SAMPLE_GRASP_COUNT_DEFAULT):
    #assert (not are_colliding(tree, kin_cache))
    assert tool_link_name is not None, "tool_link_name should be passed to pddlstream_from_problem"
    assert tool_name is not None, "tool_name should be passed to pddlstream_from_problem"
    assert mplan is not None, "mplan should be passed to pddlstream_from_problem"

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    print('Robot:', robot)
    set_configuration(robot, pscene.combined_robot.home_pose)
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
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose)]
        for surface in fixed:
            init += [('Stackable', body, surface)]
            print("body {} - surface {}".format(body, surface))
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    for body in fixed:
        name = get_body_name(body)
        if 'sink' in name:
            init += [('Sink', body)]
        if 'stove' in name:
            init += [('Stove', body)]

    body = movable[0]
    goal = ('and',
            ('AtConf', conf),
            # ('Holding', body),
            ('On', body, fixed[0]),
            # ('On', body, fixed[2]),
            # ('Cleaned', body),
            #             ('Cooked', body),
            )

    body_subject_map = make_body_subject_map(pscene, body_names)
    body_actor_map = make_body_actor_map(pscene, body_names)
    actor = pscene.actor_dict[tool_name]
    update_grasp_info({tool_name: GraspInfo(
        lambda body: sample_grasps(body_subject_map=body_subject_map, body=body, actor=actor,
                                   sample_count=grasp_sample),
        approach_pose=Pose(0.05 * Point(z=-1)))})

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen_rnb(body_subject_map, body_actor_map,
                                                      pscene.combined_robot.home_dict, fixed)),
        'sample-grasp': from_gen_fn(get_grasp_gen(robot, tool_link_name=tool_link_name, grasp_name=tool_name)),
        'inverse-kinematics': from_fn(get_ik_fn_rnb(
            pscene, body_subject_map, pscene.actor_dict[tool_name], checkers, pscene.combined_robot.home_dict,
            disabled_collisions = get_disabled_collisions(pscene.gscene, robot),
            robot=robot, fixed=fixed, teleport=teleport)),
        # 'plan-free-motion': from_fn(get_free_motion_gen_ori(robot, fixed, teleport)),
        # 'plan-holding-motion': from_fn(get_holding_motion_gen_ori(robot, fixed, teleport)),
        'plan-free-motion': from_fn(get_free_motion_gen_rnb(mplan, body_subject_map, robot, tool_link=tool_link_name)),
        'plan-holding-motion': from_fn(
            get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool_link=tool_link_name)),
        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test()),
        'test-cfree-approach-pose': from_test(get_cfree_obj_approach_pose_test()),
        'test-cfree-traj-pose': from_test(negate_test(get_movable_collision_test())),  # get_cfree_traj_pose_test()),

        'TrajCollision': get_movable_collision_test(),
    }

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def postprocess_plan(plan):
    paths = []
    for name, args in plan:
        if name == 'place':
            paths += args[-1].reverse().body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            paths += args[-1].body_paths
    return Command(paths)


