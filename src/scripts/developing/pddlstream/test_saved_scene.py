from __future__ import print_function
import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "src"))

from pkg.controller.combined_robot import *
from pkg.utils.utils import get_now, try_mkdir
import argparse

parser = argparse.ArgumentParser(description='Test saved data.')
parser.add_argument('--rtype', type=str, help='robot type name')
parser.add_argument('--dat_root', type=str, help='data root directory name', default="stowing")
parser.add_argument('--res_root', type=str, help='result root directory name', default="stowing-result")
parser.add_argument('--dat_dir', type=str, help='data folder name')
parser.add_argument('--file_option', type=str, help='data file name option')
parser.add_argument('--data_idx', type=int, help='data file index')
parser.add_argument('--cname', type=str, help='checker type')
parser.add_argument('--GRASP_SAMPLE', type=int, default=100, help='max. number of grasp to sample for a grasping instacee')
parser.add_argument('--STABLE_SAMPLE', type=int, default=100, help='max. number of stable point to sample for a placement instacee')
parser.add_argument('--VISUALIZE', type=str2bool, default=False, help='to show in RVIZ')
parser.add_argument('--PLAY_RESULT', type=str2bool, default=False, help='to play result')
parser.add_argument('--TIMEOUT_MOTION', type=int, default=5, help='motion planning timeout')
parser.add_argument('--MAX_TIME', type=int, default=100, help='TAMP timeout')
parser.add_argument('--MAX_ITER', type=int, default=100, help='TAMP max iteration')
parser.add_argument('--SHOW_STATE', type=str2bool, default=False, help='show intermediate states')
parser.add_argument('--MAX_SKELETONS', type=int, default=30, help='maximum number of skeletons to consider')
parser.add_argument('--SEARCH_SAMPLE_RATIO', type=int, default=10, help='the desired ratio of sample time / search time when max_skeletons!=None')

args = parser.parse_args()
rtype = args.rtype
dat_root = args.dat_root
res_root = args.res_root
dat_dir = args.dat_dir
file_option = args.file_option
data_idx = args.data_idx
cname = args.cname
GRASP_SAMPLE = args.GRASP_SAMPLE
STABLE_SAMPLE = args.STABLE_SAMPLE
VISUALIZE = args.VISUALIZE
PLAY_RESULT = args.PLAY_RESULT
TIMEOUT_MOTION = args.TIMEOUT_MOTION
MAX_TIME = args.MAX_TIME
SHOW_STATE = args.SHOW_STATE
MAX_ITER = args.MAX_ITER
MAX_SKELETONS=args.MAX_SKELETONS
SEARCH_SAMPLE_RATIO=args.SEARCH_SAMPLE_RATIO


DATA_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data")
try_mkdir(DATA_PATH)

TEST_DATA_PATH = os.path.join(DATA_PATH, dat_root)
try_mkdir(TEST_DATA_PATH)

TEST_RESULT_PATH = os.path.join(DATA_PATH, res_root)
try_mkdir(TEST_RESULT_PATH)

CLEARANCE = 1e-3
TOOL_RPY = (-np.pi / 2, 0, 0)

ROBOT_TYPE = {e.name: e for e in RobotType}[rtype]

if ROBOT_TYPE in [RobotType.indy7, RobotType.indy7gripper]:
    ROBOT_NAME = "indy0"
    TOOL_LINK = "indy0_tcp"
    TOOL_NAME = "grip0"
    TOOL_XYZ = (0, 0, 0.14)
    HOME_POSE = (0, 0, 0, 0, 0, 0)
    GRIP_DEPTH = 0.05

elif ROBOT_TYPE == RobotType.panda:
    ROBOT_NAME = "panda0"
    TOOL_LINK = "panda0_hand"
    TOOL_NAME = "grip0"
    TOOL_XYZ = (0, 0, 0.112)
    HOME_POSE = (0, -0.3, 0, -0.5, 0, 2.5, 0)
    GRIP_DEPTH = 0.03

from pkg.project_config import *

crob = CombinedRobot(robots_on_scene=[
    RobotConfig(0, ROBOT_TYPE, None,
                INDY_IP)]
    , connection_list=[False])

from pkg.geometry.builder.scene_builder import SceneBuilder

# for data_idx in range(100):
s_builder = SceneBuilder(None)

# xyz_rpy_robots = s_builder.detect_items(level_mask=[DetectionLevel.ROBOT])
xyz_rpy_robots = {ROBOT_NAME: ((0, 0, 0), (0, 0, 0))}
crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)
gscene = s_builder.create_gscene(crob, start_rviz=VISUALIZE)
HOME_DICT = list2dict(HOME_POSE, gscene.joint_names)

from pkg.geometry.geometry import *

gtems_robot = s_builder.add_robot_geometries(color=(0, 1, 0, 0.5), display=False, collision=True)

from pkg.planning.scene import PlanningScene

pscene = PlanningScene(gscene, combined_robot=crob)

from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlacePlane, SweepTool

gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name=TOOL_LINK,
                   dims=(0.01,) * 3, center=TOOL_XYZ, rpy=TOOL_RPY, color=(1, 0, 0, 1), display=True, collision=False,
                   fixed=True)
gripper = pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0, 0, 0), rpy=(0, 0, 0))

from pkg.planning.motion.moveit.moveit_planner import MoveitPlanner

mplan = MoveitPlanner(pscene)

from pkg.planning.constraint.constraint_subject import CustomObject, Grasp2Point, PlacePoint, SweepPoint, SweepTask
from pkg.planning.filtering.lattice_model.scene_building import *


ROBOT_DATA_ROOT = os.path.join(TEST_DATA_PATH, ROBOT_TYPE.name)
try_mkdir(ROBOT_DATA_ROOT)

ROBOT_RESULT_ROOT = os.path.join(TEST_RESULT_PATH, ROBOT_TYPE.name)
try_mkdir(ROBOT_RESULT_ROOT)

DATASET_PATH = os.path.join(ROBOT_DATA_ROOT, dat_dir)
RESULTSET_PATH = os.path.join(ROBOT_RESULT_ROOT, dat_dir)
# DATASET_PATH = os.path.join(ROBOT_DATA_ROOT, '20210223-051658')
try_mkdir(RESULTSET_PATH)
print("")
print("DATASET_PATH: {}".format(DATASET_PATH))
print("RESULTSET_PATH: {}".format(RESULTSET_PATH))
print("")

os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/developing/pddlstream'))
from convert_pscene import *
from plan_rnb import *

connect_notebook(use_gui=False)
urdf_pybullet_path = copy_meshes(gscene)

if cname == "None":
    checkers = []
if cname == "Tool":
    from pkg.planning.filtering.grasp_filter import GraspChecker

    gcheck = GraspChecker(pscene)
    checkers = [gcheck]
if cname == "ToolReach":
    from pkg.planning.filtering.grasp_filter import GraspChecker

    gcheck = GraspChecker(pscene)
    from pkg.planning.filtering.reach_filter import ReachChecker

    rcheck = ReachChecker(pscene)
    checkers = [gcheck, rcheck]
if cname == "Full":
    from pkg.planning.filtering.grasp_filter import GraspChecker

    gcheck = GraspChecker(pscene)
    from pkg.planning.filtering.reach_filter import ReachChecker

    rcheck = ReachChecker(pscene)
    from pkg.planning.filtering.latticized_filter import LatticedChecker

    lcheck = LatticedChecker(pscene, gcheck)
    checkers = [gcheck, rcheck, lcheck]
if cname == "Pairwise":
    from pkg.planning.filtering.pair_svm import PairSVM

    pcheck = PairSVM(pscene)
    checkers = [pcheck]

gtimer = GlobalTimer.instance()
fname = "data_%s_%02d.pkl" % (file_option, data_idx)
saved_data = load_pickle(os.path.join(DATASET_PATH, fname))
gtem_args = saved_data['gtem_args']
obj_args = saved_data['obj_args']
gtem_remove = []
for gtem in gscene:
    if gtem.link_name == "base_link" and gtem.parent is None:
        gtem_remove.append(gtem)
for gtem in gtem_remove:
    gscene.remove(gtem)

gid_list = np.arange(len(gtem_args)).tolist()
for gidx in gid_list:
    args = gtem_args[gidx]
    if args['parent'] is not None:
        if args['parent'] not in gscene.NAME_DICT:
            gid_list.append(gidx)
            continue
    gscene.create_safe(**args)
pscene.create_binder(bname="wp", gname="wp", _type=PlacePlane, point=None)
pscene.create_binder(bname="gp", gname="gp", _type=PlacePlane, point=None)

obj_set_list = []
for obj_name in sorted(obj_args.keys()):
    obj, obj_arg = DummyObject(), obj_args[obj_name]
    obj.name = obj_name
    obj.GRIP_DEPTH = obj_arg["GRIP_DEPTH"]
    obj.DIM = obj_arg["DIM"]
    obj.geometry = gscene.NAME_DICT[obj_arg["gname"]]
    obj.CLEARANCE = obj_arg["CLEARANCE"]
    obj_pscene, handles = add_object(pscene, obj)
    obj_set_list.append((obj, obj_pscene, handles))

obj, obj_pscene, handles = obj_set_list[0]
obj_pscene.geometry.color = (0.8, 0.2, 0.2, 1)
if VISUALIZE:
    gscene.set_rviz()

initial_state = pscene.initialize_state(crob.home_pose)
pscene.set_object_state(initial_state)
from_state = initial_state.copy(pscene)

mplan.motion_filters = checkers
checkers_ik=[checker for checker in checkers if checker.BEFORE_IK]
# checkers_ik=[checker for checker in checkers]
pscene.set_object_state(initial_state)
gscene.update()

reset_pybullet()
robot_body, body_names, movable_bodies = pscene_to_pybullet(
    pscene, urdf_pybullet_path, tool_name=TOOL_NAME, name_exclude_list=[ROBOT_NAME])
print('Objects:', body_names)
saver = WorldSaver()
mplan.reset_log(True)
problem, ik_fun = pddlstream_from_problem_rnb(pscene, robot_body, body_names=body_names,
                                      Q_init=HOME_POSE,
                                      goal_pairs=[(obj_pscene.oname, 'gp')],
                                      movable=movable_bodies,
                                      checkers=checkers_ik,
                                      tool_name=TOOL_NAME, tool_link_name=TOOL_LINK,
                                      mplan=mplan, timeout=TIMEOUT_MOTION,
                                      grasp_sample=GRASP_SAMPLE, stable_sample=STABLE_SAMPLE,
                                      show_state=SHOW_STATE)
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
                         max_skeletons=MAX_SKELETONS, search_sample_ratio=SEARCH_SAMPLE_RATIO)
        elapsed = gtimer.toc("plan") / 1000
        saver.restore()
print_solution(solution)
plan, cost, evaluations = solution
res = not any(plan is status for status in [None, False])
move_num = len(plan) if res else 0
plan_try = len(mplan.result_log["filter_fin"])
plan_num = len(mplan.result_log["planning"])
fail_num = np.sum(np.logical_not(mplan.result_log["planning"]))
sample = {"plan_time": elapsed, "length": move_num,
          "IK_tot": ik_fun.checkout_count+ik_fun.pass_count+ik_fun.fail_count,
          "IK_count": ik_fun.pass_count+ik_fun.fail_count, "failed_IKs": ik_fun.fail_count,
          "MP_tot": plan_try, "MP_count": plan_num, "failed_MPs": fail_num,
          "success": res, "body_names":body_names, "plan": plan}
save_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)), sample)

print("------- Result {} ({}): {} s -------".format(fname, cname, elapsed))
print("==========================================================")
print("==========================================================")
print(gtimer)
print("==========================================================")
print("==========================================================")

if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=body_names, plan=plan, SHOW_PERIOD=0.01)

s_builder.xcustom.clear()
