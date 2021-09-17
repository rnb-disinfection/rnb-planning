from __future__ import print_function
import os

RNB_PLANNING_DIR = os.environ["RNB_PLANNING_DIR"]
os.chdir(os.path.join(RNB_PLANNING_DIR, 'src'))
from pkg.controller.combined_robot import *
from pkg.utils.utils import get_now, try_mkdir
from pkg.planning.pddlstream.convert_pscene import *
from pkg.utils.test_scripts import *
from pkg.utils.plot_utils import *
import subprocess

##
# @extract experiment data
def extract_values(resdat_all, keys, fn=lambda x:x):
    dat_dict_dict = {}
    for scenario, resdat_dict in resdat_all.items():
        dat_dict = {}
        valid = False
        min_dat_len = 1e10
        for cname, resdat_list in resdat_dict.items():
            dat_list = []
            for resdat in resdat_list:
                error = False
                for key in keys:
                    if key not in resdat:
                        error = True
                        break
                    resdat = resdat[key]
                if error:
                    dat_list.append(None)
                else:
                    dat_list.append(fn(resdat))
            dat_len = len(dat_list)
            if dat_len > 0:
                valid = True
                dat_dict[cname] = dat_list
                min_dat_len = min(min_dat_len, dat_len)
        if valid:
            dat_dict_dict[scenario] = {k: v[:min_dat_len]for k, v in dat_dict.items()}
    return dat_dict_dict


def get_valid_idc_dict(resdat_all):
    succ_dict = extract_values(resdat_all, ["success"])
    valid_idc_dict = {case: np.all([succ for succ in casedict.values()], axis=0) for case, casedict in
                      succ_dict.items()}
    return valid_idc_dict


def print_results(RES_ROOT, RTYPE, DAT_DIR, CNAME_LIST=['None', 'Tool', 'ToolReach', 'Full']  # ,'Pairwise']
                  , CNAMES_EXCLUDE=['None'],
                  exp_list=['obj_1', 'obj_1_obs3', 'obj_1_obs5', 'obj_3', 'obj_3_pole', 'obj_3_hard',
                            'obj_1c_obs0', 'obj_1c_obs3', 'obj_1c_obs5', 'obj_1c_obs7',
                            'obj_3c_obs3', 'obj_3c_obs5', 'obj_3c_obs7']
                  , read_only=False
                  ):
    DATA_PATH = os.path.join(RNB_PLANNING_DIR, "data")
    try_mkdir(DATA_PATH)

    # RES_ROOT = "stowing-result"
    # RES_ROOT = "stowing-deep-result-before-ik-only2"
    # RES_ROOT = "stowing-result-pddl-fin"

    TEST_RESULT_PATH = os.path.join(DATA_PATH, RES_ROOT)
    try_mkdir(TEST_RESULT_PATH)

    HEADS = np.array(
        ["plan_time", "length", "MP_tot", "MP_count", "failed_MPs", "IK_tot", "IK_count", "failed_IKs", "success"])
    DISPS = np.array(
        ["Time", "Actions", "MP_tot", "MP trials", "MP fails", "IK_tot", "IK trials", "IK fails", "Fail(%)", "Total"])
    H2D = {}
    D2H = {}
    for head, disp in zip(HEADS, DISPS):
        H2D[head] = disp
        D2H[disp] = head

    # DAT_DIR = '20210702-152658'
    # DAT_DIR = '20210707-034434'
    # DAT_DIR = '20210709-172009'
    # DAT_DIR = '20210820-162838'

    # RTYPE = "panda"
    # DAT_DIR = '20210705-182244'
    # FILE_OPTION = "obj_1"
    # # DAT_DIR, FILE_OPTION = "20210613-022832", "obj_1"
    # # # DAT_DIR, FILE_OPTION = "20210612-221934", "obj_1"

    GRASP_SAMPLE = 100
    VISUALIZE = True
    PLAY_RESULT = False
    SHOW_STATE = False

    TIMEOUT_MOTION = 5
    MAX_TIME = 100
    MAX_ITER = 100

    ROBOT_DATA_ROOT = os.path.join(TEST_RESULT_PATH, RTYPE)
    DATASET_PATH = os.path.join(ROBOT_DATA_ROOT, DAT_DIR)
    file_list = sorted(os.listdir(DATASET_PATH))

    print("=" * 50)
    print("RES_ROOT: {}".format(RES_ROOT))
    print("DAT_DIR: {}".format(DAT_DIR))
    print("CNAME_LIST: {}".format(CNAME_LIST))
    print("=" * 50)

    resdat_all = defaultdict(lambda: defaultdict(list))
    res_dict_all = defaultdict(lambda: defaultdict(list))
    resfile_list = sorted(os.listdir(os.path.join(DATASET_PATH)))
    for resfile in resfile_list:
        if resfile.startswith("."):
            continue
        args = resfile[:-4].split("_")[1:]
        f_option = "_".join(args[:-2])
        fid, cname = args[-2:]
        resdat = load_pickle(os.path.join(DATASET_PATH, resfile))
        resdat_all[f_option][cname].append(resdat)
        res_dict_all[f_option][cname].append([resdat[head] if head in resdat else None for head in HEADS])
    if read_only:
        return resdat_all, res_dict_all

    for ekey in exp_list:
        print(ekey)
        if ekey not in res_dict_all:
            continue

        ## Check result file sanity
        fnames_dict = {}
        imax_min = None
        for cname in CNAME_LIST:
            fnames_dict[cname] = [int(fname.split("_")[-2]) for fname in resfile_list
                                  if (ekey == "_".join(fname.split("_")[1:-2])
                                      and cname == fname.split("_")[-1].split(".")[0])]
            len_fnames = len(fnames_dict[cname])
            if len_fnames > 0:
                imax = max(fnames_dict[cname])
                imax_min = min([imax, imax_min]) if imax_min is not None else imax
                print("{} files: {}".format(cname, len_fnames))

        ## Print missing file if any
        missing_any = False
        for cname in CNAME_LIST:
            if len(fnames_dict[cname]) > 0:
                idc_missing = set(np.arange(imax_min + 1)) - set(fnames_dict[cname])
                if len(idc_missing) > 0:
                    missing_any = True
                    print("{} missing {}".format(cname, idc_missing))

        if missing_any:
            print("Skip as there's missing result")
            continue

        res_dict = res_dict_all[ekey]

        succ_vec_list = []
        len_list = []
        None_cols = []
        CNAME_LIST_CUR = [cname for cname in CNAME_LIST if cname in res_dict]
        len_tests = None
        for cname in CNAME_LIST_CUR:
            vv = np.array(res_dict[cname])[:imax_min + 1]
            if len_tests is not None:
                assert len_tests == len(vv), "data length mismatch"
            else:
                len_tests = len(vv)
            if len(vv) > 0:
                None_cols.append(np.any(np.equal(vv, None), axis=0))
                succ_vec = vv[:, -1]
                succ_vec_list.append(succ_vec)
                len_list.append(len(succ_vec))
                if cname in CNAMES_EXCLUDE:
                    fail_vec = np.logical_not(succ_vec)
                    idc_fail_none = np.where(fail_vec)[0]
        print("=" * 50 + " {:<10} ".format(ekey) + "=" * 50, end="\r")
        if len(len_list) > 0:
            None_cols = np.any(None_cols, 0)
            good_cols = np.where(np.logical_not(None_cols))[0]
            idc_include = [i_cn for i_cn, cname in enumerate(CNAME_LIST_CUR) if cname not in CNAMES_EXCLUDE]
            min_len = np.min(np.array(len_list)[idc_include])
            succ_vec_all = np.all([svec[:min_len] for svec in
                                   np.array(succ_vec_list)[idc_include]], axis=0)
            idc_succ_all = np.where(succ_vec_all)[0]
            tot_cols = set(np.where([chead.endswith("tot") for chead in DISPS[good_cols]])[0])
            print("=" * 50 + " {:<10} ".format(ekey) + "=" * 50 + " valid={}".format(len(idc_succ_all)))
            print(("{:<10}:"
                   + "".join([
                        " {:>8}"
                        if not tot_cols.intersection([i_col - 1, i_col - 2])
                        else " {:>15}"
                        for i_col in range(len(good_cols))])).format(
                "Checker", *DISPS[good_cols]) + " {:>8}".format(DISPS[-1]))
            hard_vec_list = []
            for cname in CNAME_LIST_CUR:
                vv = np.array(res_dict[cname][:imax_min + 1])[:, good_cols]
                if len(vv) > 0:
                    succ_vec = vv[:, -1]
                    fail_vec = np.logical_not(succ_vec)
                    hard_vec_list.append(vv[:min_len, 0] > np.mean(vv[:min_len, 0]) * 1.0)
                    if cname in CNAMES_EXCLUDE:
                        idc_succ_now = np.where(succ_vec)[0]
                        idc_fail_none = np.where(fail_vec)[0]
                    else:
                        #                     idc_succ_now = np.where(succ_vec)[0]
                        idc_succ_now = idc_succ_all
                    print(
                        "{:<10}:".format(cname)  # Case name
                        + "".join([
                            " {:>8}".format(np.round(np.mean(vv[idc_succ_now, i_col]), 2))
                            + (""
                               if not tot_cols.intersection([i_col - 1, i_col - 2])
                               else
                               "({:>5})".format(
                                   np.round(np.mean(vv[idc_succ_now, i_col] / vv[idc_succ_now, i_col - 1] * 100), 2)))
                            for i_col in range(vv.shape[-1] - 1)])  # data values
                        + " {:>8} {:>8}".format(np.round(float(np.sum(fail_vec)) / len(vv) * 100, 2),
                                                len(vv)))  # meta data
    return resdat_all, res_dict_all


def print_debug_info(resdat_all,
                     CNAME_LIST=['None', 'Tool', 'ToolReach', 'Full'],
                     exp_list=['obj_1', 'obj_1_obs3', 'obj_1_obs5', 'obj_3', 'obj_3_pole', 'obj_3_hard',
                               'obj_1c_obs0', 'obj_1c_obs3', 'obj_1c_obs5', 'obj_1c_obs7',
                               'obj_3c_obs3', 'obj_3c_obs5', 'obj_3c_obs7']):
    print("==============================================")
    print("============== Checker Results ===============")
    for exp in exp_list:
        print("---------- {:10} ----------".format(exp))
        print("{:11} {:>10} {:>10} {:>10}".format("", "accuracy", "precision", "recall"))
        for cname in CNAME_LIST:
            pre_motion_checks = [dat['pre_motion_checks'] for dat in resdat_all[exp][cname]]
            planning_log = [dat['planning_log'] for dat in resdat_all[exp][cname]]
            assert len(pre_motion_checks) == len(planning_log), "MP reseult length mismatch"
            if len(pre_motion_checks) > 0:
                pre_motion_checks = np.concatenate(pre_motion_checks)
                planning_log = np.concatenate(planning_log)
                print("{:10}: {:>8} % {:>8} % {:>8} %".format(cname,
                                                              round(np.mean(
                                                                  np.equal(pre_motion_checks, planning_log)) * 100, 2),
                                                              round(np.mean(np.equal(pre_motion_checks, planning_log)[
                                                                                np.where(pre_motion_checks)]) * 100, 2),
                                                              round(np.mean(np.equal(pre_motion_checks, planning_log)[
                                                                                np.where(planning_log)]) * 100, 2)))

    print("==============================================")
    print("")
    print("==============================================")
    print("=============== Cache Check ==================")
    for exp in exp_list:
        print("---------- {:10} ----------".format(exp))
        print("{:11} {:>10} {:>10} {:>10}".format("", "accuracy", "precision", "recall"))
        for cname in CNAME_LIST:
            cache_log = [np.array(dat['cache_log'])[:, 0]
                         if 'cache_log' in dat else []
                         for dat in resdat_all[exp][cname]]
            ik_result = [np.array(dat['cache_log'])[:, 1]
                         if 'cache_log' in dat else []
                         for dat in resdat_all[exp][cname]]
            if len(cache_log) > 0:
                for i_c, (cache, ik) in enumerate(zip(cache_log, ik_result)):
                    if not np.all(np.equal(cache, ik)):
                        print("Wrong: {} - {}".format(i_c, np.where(np.logical_not(np.equal(cache, ik)))[0].tolist()))
                cache_log = np.concatenate(cache_log)
                ik_result = np.concatenate(ik_result)
                print("{:10}: {:>8} % {:>8} % {:>8} %".format(cname,
                                                              round(np.mean(np.equal(cache_log, ik_result)) * 100, 2),
                                                              round(np.mean(np.equal(cache_log, ik_result)[
                                                                                np.where(cache_log)]) * 100, 2),
                                                              round(np.mean(np.equal(cache_log, ik_result)[
                                                                                np.where(ik_result)]) * 100, 2)))

    print("==============================================")
    print("")
    print("==============================================")
    print("============= IK Check Results ===============")
    exp_list = ['obj_1', 'obj_1_obs3', 'obj_1_obs5', 'obj_3', 'obj_3_pole', 'obj_3_hard']
    for exp in exp_list:
        print("---------- {:10} ----------".format(exp))
        print("{:11} {:>10} {:>10} {:>10}".format("", "accuracy", "precision", "recall"))
        for cname in CNAME_LIST:
            ik_feas = [dat['ik_feas'] for dat in resdat_all[exp][cname]]
            ik_res = [dat['ik_res'] for dat in resdat_all[exp][cname]]
            assert len(ik_feas) == len(ik_res), "ik reseult length mismatch"
            if len(ik_feas) > 0:
                ik_feas = np.concatenate(ik_feas)
                ik_res = np.concatenate(ik_res)
                print("{:10}: {:>8} % {:>8} % {:>8} %".format(cname,
                                                              round(np.mean(np.equal(ik_feas, ik_res)) * 100, 2),
                                                              round(np.mean(
                                                                  np.equal(ik_feas, ik_res)[np.where(ik_feas)]) * 100,
                                                                    2),
                                                              round(np.mean(
                                                                  np.equal(ik_feas, ik_res)[np.where(ik_res)]) * 100,
                                                                    2)))
    print("==============================================")
    print("")
    print("==============================================")
    print("============= IK Filter Results ==============")

    correct_map = {
        "Success": ['success', 'col_approach', 'ik_approach'],
        "GraspChecker": ['col_approach', 'ik_approach'],
        "ReachChecker": ['ik_approach'],
    }

    ik_correct_all = []
    for exp, resdats in sorted(resdat_all.items()):
        print("=== " + exp + " ===")
        for cname, dat_list in resdats.items():
            if len(dat_list) > 0:
                print(cname)
            for i_d, dat in enumerate(dat_list):
                ik_feas_reason, ik_res_reason = dat['ik_feas_reason'], dat['ik_res_reason']
                ik_correct = [rres in correct_map[fres] for fres, rres in zip(ik_feas_reason, ik_res_reason)]
                ik_correct_all = np.concatenate([ik_correct_all, ik_correct])
                idc_wrong = np.where(np.logical_not(ik_correct))[0].tolist()
                if len(idc_wrong) > 0:
                    for idc in idc_wrong:
                        print("{} - {}: {} / {}".format(i_d, idc, ik_feas_reason[idc], ik_res_reason[idc]))
        print("average accuracy: {}".format(round(np.mean(ik_correct_all) * 100, 2)))
    print("==============================================")


import matplotlib.pyplot as plt


def plot_times(resdat_all,
               CHECKERS=['None', 'Tool', 'ToolReach', 'Full'],
               CASES=['obj_1', 'obj_1_obs3', 'obj_1_obs5', 'obj_3', 'obj_3_pole', 'obj_3_hard',
                      'obj_1c_obs0', 'obj_1c_obs3', 'obj_1c_obs5', 'obj_1c_obs7',
                      'obj_3c_obs3', 'obj_3c_obs5', 'obj_3c_obs7']):
    valid_idc_dict = get_valid_idc_dict(resdat_all)

    plt.figure(figsize=(15, 12))
    plt.subplot(2, 1, 1)
    time_dict = {case:
                     {cname: np.array(tvec)[valid_idc_dict[case][:len(tvec)]]
                      for cname, tvec in tdict.items()}
                 for case, tdict in extract_values(resdat_all, ["plan_time"]).items()}
    groups, cnames = grouped_bar(time_dict, groups=CASES, cases=CHECKERS)
    plt.title("mean planning time")

    plt.subplot(2, 3, (4, 5))
    time_max = 0
    grouped_bar(time_dict, groups=CASES, cases=CHECKERS, scatter=True)
    plt.title("scattered planning times")

    plt.subplot(2, 3, 6)
    time_max = 0
    for xsmall, cname in enumerate(cnames):
        times = [time_dict[group][cname] for group in groups]
        times = np.concatenate(times)
        stds = np.std(times)
        times = np.mean(times)
        plt.bar(xsmall + 0.5, times, yerr=stds)
        time_max = max(time_max, times + stds)
    plt.axis([0, len(cnames), 0, time_max + 1])
    plt.grid()
    plt.xticks(np.arange(len(cnames)) + 1, cnames)
    plt.title("total mean planning time")


def plot_log(
        resdat_all,
        keys,
        fn=np.mean,
        CHECKERS=['None', 'Tool', 'ToolReach', 'Full'],
        CASES=['obj_1c_obs0', 'obj_1c_obs3', 'obj_1c_obs5', 'obj_1c_obs7', 'obj_3c_obs3', 'obj_3c_obs5', 'obj_3c_obs7'],
        scatter=False
):
    valid_idc_dict = get_valid_idc_dict(resdat_all)
    time_dict = {case:
                     {cname: np.array(tvec)[valid_idc_dict[case][:len(tvec)]]
                      for cname, tvec in tdict.items()}
                 for case, tdict in extract_values(resdat_all, keys, fn=fn).items()}
    grouped_bar(time_dict, groups=CASES, cases=CHECKERS, scatter=scatter)
    return time_dict