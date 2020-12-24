from pkg.planner.moveit.moveit_py import MoveitCompactPlanner_BP, ObjectType, ObjectMPC

from pkg.utils.utils import *
gtimer = GlobalTimer.instance()

urdf_path, srdf_path = './moveit_plan_compact/test_assets/custom_robots.urdf', './moveit_plan_compact/test_assets/custom_robots.srdf'

planner = MoveitCompactPlanner_BP(urdf_path, srdf_path, ["indy0", "indy0_tcp"])

planner = MoveitCompactPlanner_BP(urdf_path, srdf_path, ["indy0", "indy0_tcp"])