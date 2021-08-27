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

args = parser.parse_args()
rtype = args.rtype
dat_root = args.dat_root
res_root = args.res_root
dat_dir = args.dat_dir
file_option = args.file_option
data_idx = args.data_idx
cname = args.cname
VISUALIZE = True
PLAY_RESULT = True

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

pscene.set_object_state(initial_state)
gscene.update()

sample = load_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)))

res = sample["success"]
plan = sample["plan"]
body_names = sample["body_names"]

if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=body_names, plan=plan, SHOW_PERIOD=0.01)

s_builder.xcustom.clear()
