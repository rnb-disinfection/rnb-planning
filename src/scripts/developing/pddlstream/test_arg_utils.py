import sys
import os
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "src"))
from pkg.utils.utils import *
import argparse

def parse_test_args():
    parser = argparse.ArgumentParser(description='Test saved data.')
    parser.add_argument('--rtype', type=str, help='robot type name')
    parser.add_argument('--dat_root', type=str, help='data root directory name')
    parser.add_argument('--res_root', type=str, help='result root directory name')
    parser.add_argument('--dat_dir', type=str, help='data folder name')
    parser.add_argument('--file_option', type=str, help='data file name option')
    parser.add_argument('--data_idx', type=int, help='data file index')
    parser.add_argument('--cname', type=str, help='checker type', default="None")

    parser.add_argument('--TIMEOUT_MOTION', type=float, default=5, help='motion planning timeout')
    parser.add_argument('--IK_TRY_NUM', type=int, default=10, help='max. number of trials for ik')
    parser.add_argument('--IK_TIMEOUT_SINGLE', type=float, default=0.01, help='timeout for single ik trial')
    parser.add_argument('--MAX_TIME', type=int, default=100, help='TAMP timeout')
    parser.add_argument('--MAX_ITER', type=int, default=100, help='TAMP max iteration')
    parser.add_argument('--MAX_SKELETONS', type=int, default=30, help='maximum number of skeletons to consider')

    parser.add_argument('--GRASP_SAMPLE', type=int, default=100, help='max. number of grasp to sample for a grasping instacee')
    parser.add_argument('--STABLE_SAMPLE', type=int, default=100, help='max. number of stable point to sample for a placement instacee')
    parser.add_argument('--SEARCH_SAMPLE_RATIO', type=int, default=10, help='the desired ratio of sample time / search time when max_skeletons!=None')

    parser.add_argument('--VISUALIZE', type=str2bool, default=False, help='to show in RVIZ')
    parser.add_argument('--SHOW_STATE', type=str2bool, default=False, help='show intermediate states')
    parser.add_argument('--PLAY_RESULT', type=str2bool, default=False, help='to play result')
    parser.add_argument('--USE_PYBULLET_GUI', type=str2bool, default=False, help='to show in pybullet gui')
    parser.add_argument('--USE_MOVEIT_IK', type=str2bool, default=True, help='to use ik solving by moveit (no random initialization)')
    parser.add_argument('--TIMED_COMPLETE', type=str2bool, default=False, help='to use timed ik solving for infeasible predictions')

    parser.add_argument('--VERBOSE', type=str2bool, default=False, help='to print states')
    parser.add_argument('--STACK_TIMELOG', type=str2bool, default=False, help='to stack all time logs, not average')
    parser.add_argument('--SAVE_RESULTS', type=str2bool, default=True, help='to save results (overwrite same index)')

    args = parser.parse_args()
    print("=================== arguments ==================")
    for k,v in vars(args).items():
        print("{}: {}".format(k, v))
    return args
    print("================================================")


def get_checkers_by_case_name(cname, pscene):
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
    return checkers