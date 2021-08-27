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


CLEARANCE = 1e-3
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
ROBOT_NAME, TOOL_LINK, TOOL_XYZ, HOME_POSE, GRIP_DEPTH = get_single_robot_params(ROBOT_TYPE)
s_builder, pscene = prepare_single_robot_scene(ROBOT_TYPE, ROBOT_NAME, TOOL_LINK, TOOL_XYZ, VISUALIZE=VISUALIZE)
crob, gscene = pscene.combined_robot, pscene.gscene

fname = "data_%s_%02d.pkl" % (file_option, data_idx)
file_gtems = os.path.join(DATASET_PATH, fname)
initial_state = load_saved_scene(pscene, file_gtems, VISUALIZE=VISUALIZE)

mplan = MoveitPlanner(pscene)
checkers = get_checkers_by_case_name(cname, pscene)

mplan.motion_filters = checkers


########################################################
#################### Solve problem  ####################
obj_pscene = pscene.subject_dict[pscene.subject_name_list[0]]
obj_pscene.geometry.color = (0.8, 0.2, 0.2, 1)
goal_pairs=[(obj_pscene.oname, 'gp')]

gtimer = GlobalTimer.instance()
gtimer.reset()

res, plan, log_dict = solve_in_pddlstream(pscene, mplan, ROBOT_NAME, TOOL_NAME, HOME_POSE, goal_pairs,
                        TIMEOUT_MOTION, MAX_TIME, MAX_ITER, MAX_SKELETONS,
                        GRASP_SAMPLE, STABLE_SAMPLE, SHOW_STATE, SEARCH_SAMPLE_RATIO,
                        use_pybullet_gui=False)

save_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)), log_dict)

print("------- Result {} ({}): {} s -------".format(fname, cname, log_dict["plan_time"]))
print("==========================================================")
print("==========================================================")
print(gtimer)
print("==========================================================")
print("==========================================================")

if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=body_names, plan=plan, SHOW_PERIOD=0.01)

s_builder.xcustom.clear()