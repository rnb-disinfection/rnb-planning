from __future__ import print_function
import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "src"))

from pkg.controller.combined_robot import *
from pkg.utils.utils import get_now, try_mkdir
import argparse

parser = argparse.ArgumentParser(description='Test saved data.')
parser.add_argument('--rtype', type=str, help='robot type name')
parser.add_argument('--dat_dir', type=str, help='data folder name')
parser.add_argument('--file_option', type=str, help='data file name option')
parser.add_argument('--data_idx', type=int, help='data file index')
parser.add_argument('--cname', type=str, help='checker type')
parser.add_argument('--GRASP_SAMPLE', type=int, default=10, help='number of grasp to sample at once')
parser.add_argument('--VISUALIZE', type=str2bool, default=False, help='to show in RVIZ')
parser.add_argument('--PLAY_RESULT', type=str2bool, default=False, help='to play result')
parser.add_argument('--TIMEOUT_MOTION', type=int, default=5, help='motion planning timeout')
parser.add_argument('--MAX_TIME', type=int, default=100, help='TAMP timeout')
parser.add_argument('--SHOW_STATE', type=str2bool, default=False, help='show intermediate states')

args = parser.parse_args()
rtype = args.rtype
dat_dir = args.dat_dir
file_option = args.file_option
data_idx = args.data_idx
cname = args.cname
GRASP_SAMPLE = args.GRASP_SAMPLE
VISUALIZE = args.VISUALIZE
PLAY_RESULT = args.PLAY_RESULT
TIMEOUT_MOTION = args.TIMEOUT_MOTION
MAX_TIME = args.MAX_TIME
SHOW_STATE = args.SHOW_STATE

DATA_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data")
try_mkdir(DATA_PATH)

TEST_DATA_PATH = os.path.join(DATA_PATH, "stowing")
try_mkdir(TEST_DATA_PATH)

TEST_RESULT_PATH = os.path.join(DATA_PATH, "stowing-result")
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


##
# @class WorkPlane
# @brief working plane. target and obstacle objects are generated on this plane
class WorkPlane(ObstacleBase):
    RTH_MIN = (0.3, -np.pi / 2, -0.1)
    RTH_MAX = (0.5, +np.pi / 2, +0.4)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.4, 0.6, 0.05)
    DIM_MAX = (0.4, 0.6, 0.05)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.8, 0.8, 0.2, 1)

    def __init__(self, gscene, name, floor_height=None, *args, **kwargs):
        assert floor_height is not None, "floor_height needed"
        if floor_height > self.RTH_MIN[2]:
            self.RTH_MIN = self.RTH_MIN[:2] + (floor_height,)
        self.H = 0.4
        ObstacleBase.__init__(self, gscene, name, *args, **kwargs)

    def is_overlapped_with(self, gtem):
        verts, radii = gtem.get_vertice_radius()
        verts_global = np.add(np.matmul(verts, gtem.orientation_mat.transpose()), gtem.center)
        verts_wp = np.multiply(DEFAULT_VERT_DICT[self.GTYPE], tuple(self.DIM[:2]) + (self.H,))
        verts_wp_global = np.add(np.matmul(verts_wp, self.geometry.orientation_mat.transpose()),
                                 np.add(self.geometry.center, (0, 0, self.H / 2)))
        return get_gjk_distance(get_point_list(verts_global), get_point_list(verts_wp_global)) - radii < 1e-4


##
# @class Box
# @brief box with the top and the front side open
class Box(WorkPlane):
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.6, 0.6, 0.05)
    COLOR = (0.8, 0.8, 0.2, 0.5)
    H_RANGE = (0.3, 0.6)
    THICKNESS = 0.05

    def __init__(self, gscene, name, H=None, **kwargs):
        WorkPlane.__init__(self, gscene=gscene, name=name, **kwargs)
        self.H = np.random.uniform(*self.H_RANGE) if H is None else H

        ## back wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_bw", link_name="base_link",
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(self.DIM[0] / 2 + self.THICKNESS / 2, 0, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## left wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_lw", link_name="base_link",
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, -self.DIM[1] / 2 - self.THICKNESS / 2, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## right wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_rw", link_name="base_link",
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, self.DIM[1] / 2 + self.THICKNESS / 2, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))


##
# @class SideBox
# @brief box with a side face open
class SideBox(Box):
    RTH_MIN = (0.3, -np.pi / 2, -0.1)
    RTH_MAX = (0.5, +np.pi / 2, +0.4)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.4, 0.6, 0.05)
    DIM_MAX = (0.4, 0.6, 0.05)
    COLOR = (0.2, 0.2, 0.8, 1)
    H_RANGE = (0.4, 0.4)
    THICKNESS = 0.05

    def __init__(self, gscene, name, **kwargs):
        Box.__init__(self, gscene=gscene, name=name, **kwargs)

        ## top
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_tp", link_name="base_link",
            dims=(self.DIM[0], self.DIM[1], self.THICKNESS), center=(0, 0, self.H + self.THICKNESS / 2), rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))


##
# @class TopBox
# @brief box with the top face open
class TopBox(Box):
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.6, 0.6, 0.05)
    H_RANGE = (0.3, 0.6)

    def __init__(self, gscene, name, **kwargs):
        Box.__init__(self, gscene=gscene, name=name, **kwargs)

        ## front wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_fw", link_name="base_link",
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(-self.DIM[0] / 2 - self.THICKNESS / 2, 0, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))


##
# @class Floor
# @brief Floor - lowerbound of the workspace
class Floor(ObstacleBase):
    RTH_MIN = (0.0, 0, -0.5)
    RTH_MAX = (0.0, 0, -0.5)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (3, 3, 0.1)
    DIM_MAX = (3, 3, 0.1)
    GTYPE = GEOTYPE.BOX


##
# @class Pole
# @brief occasional poles that obstruct robot motion
class Pole(ObstacleBase):
    RTH_MIN = (0.3, -np.pi, 0)
    RTH_MAX = (0.3, +np.pi, 0)
    RPY_MIN = (0, 0, -np.pi / 6)
    RPY_MAX = (0, 0, +np.pi / 6)
    DIM_MIN = (0.1, 0.1, 4)
    DIM_MAX = (0.1, 0.1, 4)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.7, 0.7, 0.7, 0.3)


##
# @class PlaneObstacle
# @brief Obstacles on the workplane
class PlaneObject(ObstacleBase):
    RTH_MIN = (0.3, -np.pi / 2, -0.2)
    RTH_MAX = (0.8, +np.pi / 2, +0.5)
    RPY_MIN = (0, 0, -np.pi)
    RPY_MAX = (0, 0, +np.pi)
    DIM_MIN = (0.05, 0.1, 0.2)
    DIM_MAX = (0.05, 0.1, 0.2)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.2, 0.8, 0.2, 1)

    def __init__(self, gscene, name, workplane, GRIP_DEPTH, CLEARANCE, XYZ_LOC=None, **kwargs):
        self.GRIP_DEPTH = GRIP_DEPTH
        ObstacleBase.__init__(self, gscene=gscene, name=name, **kwargs)
        verts, radii = self.geometry.get_vertice_radius()
        verts_rot = np.matmul(self.geometry.orientation_mat, verts.transpose())  ## verices with global orientaion
        verts_rot_loc = np.matmul(workplane.geometry.Toff[:3, :3].transpose(),
                                  verts_rot)  ## verices with local orientaion
        max_verts = np.max(verts_rot_loc, axis=-1)
        min_verts = np.min(verts_rot_loc, axis=-1)
        if XYZ_LOC is None:
            self.XYZ_LOC = np.random.uniform(np.negative(workplane.DIM) / 2 - min_verts + radii,
                                             np.array(workplane.DIM) / 2 - max_verts - radii)
            self.XYZ_LOC[2] = workplane.DIM[2] / 2 + self.DIM[2] / 2 + CLEARANCE
        else:
            self.XYZ_LOC = self.XYZ_LOC
        self.XYZ = np.matmul(workplane.geometry.Toff[:3, :3], self.XYZ_LOC) + workplane.geometry.Toff[:3, 3]
        self.geometry.set_offset_tf(center=self.XYZ)
        self.RTH = cart2cyl(*self.XYZ)
        gscene.update_marker(self.geometry)


def disperse_objects(gscene, object_class, key, Nmax, workplane_on, GRIP_DEPTH, CLEARANCE=1e-3):
    clear_class(gscene, key, Nmax)

    obs_list = []
    while len(obs_list) < Nmax:
        iw = len(obs_list)
        obs = object_class(gscene, "{}_{}".format(key, iw), workplane_on, GRIP_DEPTH=GRIP_DEPTH, CLEARANCE=CLEARANCE)
        remove_this = False
        for obs_pre in obs_list:
            if obs_pre.is_overlapped_with(obs.geometry):
                remove_this = True
                break
        if remove_this:
            gscene.remove(obs.geometry)
        else:
            obs_list.append(obs)
    return obs_list


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
saved_data = load_pickle(os.path.join(DATASET_PATH, "data_%s_%02d.pkl" % (file_option, data_idx)))
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
pscene.set_object_state(initial_state)
gscene.update()

reset_pybullet()
robot_body, body_names, movable_bodies = pscene_to_pybullet(
    pscene, urdf_pybullet_path, tool_name=TOOL_NAME, name_exclude_list=[ROBOT_NAME])
print('Objects:', body_names)
saver = WorldSaver()
mplan.reset_log(True)
problem = pddlstream_from_problem_rnb(pscene, robot_body, body_names=body_names,
                                      Q_init=HOME_POSE,
                                      goal_pairs=[(obj_pscene.oname, 'gp')],
                                      movable=movable_bodies,
                                      checkers=checkers_ik,
                                      tool_name=TOOL_NAME, tool_link_name=TOOL_LINK,
                                      mplan=mplan, timeout=TIMEOUT_MOTION, grasp_sample=GRASP_SAMPLE,
                                      show_state=SHOW_STATE)
_, _, _, stream_map, init, goal = problem
print('Init:', init)
print('Goal:', goal)
print('Streams:', str_from_object(set(stream_map)))
with Profiler():
    with LockRenderer(lock=not True):
        gtimer.tic("plan")
        solution = solve(problem, algorithm='adaptive',
                         unit_costs=False, success_cost=INF, max_time=MAX_TIME)
        elapsed = gtimer.toc("plan") / 1000
        saver.restore()
print_solution(solution)
plan, cost, evaluations = solution
res = not any(plan is status for status in [None, False])
move_num = len(plan) if res else 0
plan_num = len(mplan.result_log["planning"])
fail_num = np.sum(np.logical_not(mplan.result_log["planning"]))
sample = {"plan_time": elapsed, "length": move_num, "MP_count": plan_num, "failed_MPs": fail_num, "success": res}
save_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)), sample)

print("------- Result ({}): {} s -------".format(cname, elapsed))

if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=body_names, plan=plan, SHOW_PERIOD=0.1)

s_builder.xcustom.clear()
