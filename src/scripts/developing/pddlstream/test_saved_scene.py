from __future__ import print_function
import os
import sys
from test_arg_utils import *

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
MAX_SKELETONS = args.MAX_SKELETONS
SEARCH_SAMPLE_RATIO = args.SEARCH_SAMPLE_RATIO
USE_PYBULLET_GUI = args.USE_PYBULLET_GUI
SAVE_RESULTS = args.SAVE_RESULTS
USE_MOVEIT_IK = args.USE_MOVEIT_IK
TIMED_COMPLETE = args.TIMED_COMPLETE
STACK_TIMELOG= args.STACK_TIMELOG
IK_TRY_NUM = args.IK_TRY_NUM
IK_TIMEOUT_SINGLE = args.IK_TIMEOUT_SINGLE
VERBOSE = args.VERBOSE

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
ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, HOME_POSE, GRIP_DEPTH = get_single_robot_params(ROBOT_TYPE)
s_builder, pscene = prepare_single_robot_scene(ROBOT_TYPE, ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, VISUALIZE=VISUALIZE)
crob, gscene = pscene.combined_robot, pscene.gscene
crob.home_pose = HOME_POSE
crob.home_dict = list2dict(HOME_POSE, gscene.joint_names)

fname = "data_%s_%02d.pkl" % (file_option, data_idx)
print(fname)
set_meta_data("dat_root", dat_root)
set_meta_data("rtype", rtype)
set_meta_data("dat_dir", dat_dir)
set_meta_data("fname", fname)
from pkg.planning.pddlstream import convert_pscene
convert_pscene.RECORD_MODE = True


file_gtems = os.path.join(DATASET_PATH, fname)
initial_state = load_saved_scene(pscene, file_gtems, VISUALIZE=VISUALIZE)

gscene.NAME_DICT['obj_0'].color = (1,0,0,1)
gscene.update_markers_all()

mplan = MoveitPlanner(pscene)
checkers = get_checkers_by_case_name(cname, pscene)

mplan.motion_filters = checkers

###################################################
################ Manual Adjustments ###############
# gscene.NAME_DICT["pole"].collision = False


########################################################
#################### Solve problem  ####################
obj_pscene = pscene.subject_dict[pscene.subject_name_list[0]]
obj_pscene.geometry.color = (0.8, 0.2, 0.2, 1)
gscene.update_markers_all()
goal_pairs=[(obj_pscene.oname, 'gp')]

gtimer = GlobalTimer.instance()
gtimer.reset(stack=STACK_TIMELOG)


res, plan, log_dict = solve_in_pddlstream(pscene, mplan, ROBOT_NAME, TOOL_NAME, HOME_POSE, goal_pairs,
                                          TIMEOUT_MOTION, MAX_TIME, MAX_ITER, MAX_SKELETONS,
                                          GRASP_SAMPLE, STABLE_SAMPLE, SHOW_STATE, SEARCH_SAMPLE_RATIO,
                                          use_pybullet_gui=USE_PYBULLET_GUI, USE_MOVEIT_IK=USE_MOVEIT_IK, 
                                          TIMED_COMPLETE=TIMED_COMPLETE,
                                          IK_TRY_NUM=IK_TRY_NUM, IK_TIMEOUT_SINGLE=IK_TIMEOUT_SINGLE, VERBOSE=VERBOSE)

log_dict.update(mplan.result_log)
log_dict.update(GlobalLogger.instance())

if SAVE_RESULTS:
    log_dict.update({"args": vars(args)})
    save_pickle(os.path.join(RESULTSET_PATH, "result_%s_%02d_%s.pkl" % (file_option, data_idx, cname)), log_dict)

print("==========================================================")
print("======= Result {} ({}): {} s =======".format(fname, cname, log_dict["plan_time"]))
print("==========================================================")
print(gtimer)
print("==========================================================")
print("==========================================================")


if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=log_dict['body_names'], plan=plan, SHOW_PERIOD=0.01)

s_builder.xcustom.clear()