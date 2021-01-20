import tesseract
import os
import re
import numpy as np
import time
from ....utils.utils import *
from ....utils.rotation_utils import *
from ....global_config import *

import rospkg

rospack = rospkg.RosPack()


class RosResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(RosResourceLocator, self).__init__()

    def locateResource(self, url):
        fname = None
        if "package://" in url:
            url_split = url[10:].split("/")
            pkg_name = url_split[0]
            pkg_path = rospack.get_path(pkg_name)
            fname = os.path.join(pkg_path, *url_split[1:])

            with open(fname, 'rb') as f:
                resource_bytes = f.read()
        else:
            raise (RuntimeError("unknown resource"))
            return None

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource


# base_names=['indy0_link0', 'panda1_link0'], link_names=['indy0_tcp', 'panda1_hand']
def load_custom_robot():
    with open(os.path.join(WORKING_DIR, "robots", "custom_robots.urdf"), 'r') as f:
        robot_urdf = f.read()

    with open(os.path.join(WORKING_DIR, "robots", "custom_robots.srdf"), 'r') as f:
        robot_srdf = f.read()

    tes = tesseract.Tesseract()

    tes.init(robot_urdf, robot_srdf, RosResourceLocator())
    return tes


@record_time
def plan_manipulation(tes, robot_name, end_link, Tbe, base_link="base_link"):
    pci = tesseract.ProblemConstructionInfo(tes)

    pci.init_info.type = tesseract.InitInfo.STATIONARY
    # pci.init_info.data = np.array([0.0,0,0,0,0,0])

    pci.basic_info.n_steps = 50
    pci.basic_info.manip = robot_name
    pci.basic_info.start_fixed = True
    pci.basic_info.use_time = False
    pci.basic_info.dt_upper_lim = 1
    pci.basic_info.dt_lower_lim = 0.9999

    pci.opt_info.max_iter = 200
    pci.opt_info.min_approx_improve = 1e-3
    pci.opt_info.min_trust_box_size = 1e-3

    pci.kin = pci.getManipulator(pci.basic_info.manip)

    start_pos_terminfo = tesseract.JointPosTermInfo()
    start_pos_terminfo.name = "start"
    start_pos_terminfo.term_type = tesseract.TT_COST
    # start_pos_terminfo.coeffs=np.ones(6)
    start_pos_terminfo.first_step = 0
    start_pos_terminfo.last_step = 0
    # start_pos_terminfo.lower_tols=np.ones(6)*-0.01
    # start_pos_terminfo.upper_tols=np.ones(6)*0.01

    start_pos_terminfo.targets = np.array([0.0] * 6, dtype=np.float64)

    end_pos_terminfo = tesseract.CartPoseTermInfo()
    xyz = Tbe[:3, 3]
    wxyz = Rotation.from_dcm(Tbe[:3, :3]).as_quat()[[0, 1, 2, 3]]
    jsonval = \
        """
        {{
            "type" : "cart_pose",
            "params": {{
              "xyz" : [{xyz}],
              "wxyz" : [{wxyz}],
              "target" : "{target}",
              "link" : "{link}"
            }}
        }}
    
        """.format(xyz=round_it_str(xyz, 8), wxyz=round_it_str(wxyz, 8), target=base_link, link=end_link)

    end_pos_terminfo.fromJson(pci, jsonval)

    # end_pos_terminfo = tesseract.JointPosTermInfo()
    # # end_pos_terminfo = tesseract.CartPoseTermInfo()
    end_pos_terminfo.name = "end"
    end_pos_terminfo.term_type = tesseract.TT_COST
    # end_pos_terminfo.coeffs=np.ones(6)
    # end_pos_terminfo.targets=np.array([0.5]*6, dtype=np.float64)
    end_pos_terminfo.first_step = pci.basic_info.n_steps - 1
    end_pos_terminfo.last_step = pci.basic_info.n_steps - 1
    # #end_pos_terminfo.lower_tols=np.ones(6)*-0.01
    # #end_pos_terminfo.upper_tols=np.ones(6)*0.01

    joint_vel = tesseract.JointVelTermInfo()
    joint_vel.coeffs = np.ones(6)
    joint_vel.targets = np.zeros(6)
    joint_vel.first_step = 0
    joint_vel.last_step = pci.basic_info.n_steps - 1
    joint_vel.name = "Joint_vel"
    joint_vel.term_type = tesseract.TT_COST

    time_terminfo = tesseract.TotalTimeTermInfo()
    time_terminfo.coeff = 1
    time_terminfo.term_type = tesseract.TT_COST | tesseract.TT_USE_TIME
    time_terminfo.name = "time"

    collision = tesseract.CollisionTermInfo()
    collision.name = "collision"
    collision.term_type = tesseract.TT_CNT
    collision.continuous = True
    collision.first_step = 0
    collision.last_step = pci.basic_info.n_steps - 1
    collision.gap = 1
    collision_info = tesseract.createSafetyMarginDataVector(pci.basic_info.n_steps, .0001, 40)
    print(len(collision_info))
    for i in collision_info:
        collision.info.append(i)
        global ci
        ci = i

    pci.cost_infos.append(start_pos_terminfo)
    pci.cost_infos.append(end_pos_terminfo)
    # pci.cost_infos.append(time_terminfo)
    pci.cnt_infos.push_back(collision)
    pci.cost_infos.push_back(joint_vel)

    prob = tesseract.ConstructProblem(pci)
    config = tesseract.TrajOptPlannerConfig(prob)

    planner = tesseract.TrajOptMotionPlanner()

    planner.setConfiguration(config)
    planner_status_code, planner_response = planner.solve(True)
    assert planner_status_code.value() == 0, "Planning failed"

    return planner_response.joint_trajectory.trajectory


def plan_robot(tes, robot_name, binder, action_point, joint_dict, from_link='base_link'):
    Tap = action_point.get_tf_handle(joint_dict, from_link=from_link)
    Tbe = np.matmul(Tap, SE3_inv(binder.Toff_lh))
    return plan_manipulation(tes, robot_name, binder.geometry.link_name, Tbe)