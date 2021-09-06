from __future__ import print_function
import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "src"))

from pkg.utils.test_scripts import *
from pkg.planning.pddlstream.plan_rnb import *

args = parse_test_args()

rtype = args.rtype
dat_root = args.dat_root
res_root = args.res_root
dat_dir = args.dat_dir
file_option = args.file_option
data_idx = args.data_idx
cname = args.cname
VISUALIZE = True
PLAY_RESULT = True

TOOL_NAME="grip0"
ROBOT_TYPE = {e.name: e for e in RobotType}[rtype]

########################################################
################### Create data folders ################
DATASET_PATH = create_data_dirs(dat_root, rtype, dat_dir)
RESULTSET_PATH = create_data_dirs(res_root, rtype, dat_dir)
print("-"*50)
print("DATASET_PATH: {}".format(DATASET_PATH))
print("RESULTSET_PATH: {}".format(RESULTSET_PATH))
print("-"*50)

########################################################
########## Load scene and prepare planner  #############
ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, HOME_POSE, GRIP_DEPTH = get_single_robot_params(ROBOT_TYPE)
s_builder, pscene = prepare_single_robot_scene(ROBOT_TYPE, ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, VISUALIZE=VISUALIZE)
crob, gscene = pscene.combined_robot, pscene.gscene

fname = "data_%s_%02d.pkl" % (file_option, data_idx)
file_gtems = os.path.join(DATASET_PATH, fname)
initial_state = load_saved_scene(pscene, file_gtems, VISUALIZE=VISUALIZE)
gscene.NAME_DICT['obj_0'].color = (1,0,0,1)
gscene.update_markers_all()

sample = load_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)))

res = sample["success"]
plan = sample["plan"]
body_names = sample["body_names"]

for action in plan:
    print(action)

if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=body_names, plan=plan, SHOW_PERIOD=0.01)

s_builder.xcustom.clear()