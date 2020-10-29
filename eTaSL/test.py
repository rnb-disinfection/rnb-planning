from __future__ import print_function
from enum import Enum

class Scenario(Enum):
    single_object_single_robot = 0
    double_object_single_robot = 1
    single_object_dual_robot = 2
    assembly_3_piece = 3
    custom_robots = 4

current_scenario = Scenario.custom_robots

urdf_content = None
if current_scenario in [Scenario.single_object_dual_robot, Scenario.assembly_3_piece]:
    from pkg.ur10_dual import URDF_PATH, JOINT_NAMES, LINK_NAMES, ZERO_JOINT_POSE, get_geometry_items_dict
elif current_scenario in [Scenario.single_object_single_robot, Scenario.double_object_single_robot]:
    from pkg.ur10 import URDF_PATH, JOINT_NAMES, LINK_NAMES, ZERO_JOINT_POSE, get_geometry_items_dict
else:
    from pkg.robots_custom import *

    xcustom = XacroCustomizer()
    xcustom.clear()
    xcustom.add_robot(RobotType.indy7_robot, xyz=[0, -0.5, 0], rpy=[0, 0, 0])
    xcustom.add_robot(RobotType.panda_robot, xyz=[0, 0.5, 0], rpy=[0, 0, 0])
    xcustom.write_xacro()
    vel_scale = 1.0 / 2.0
    JOINT_NAMES, LINK_NAMES, ZERO_JOINT_POSE, urdf_content = \
        xcustom.convert_xacro_to_urdf(
            joint_fix_dict={'finger': 'upper'},
            vel_limit_dict={k: v * vel_scale for k, v in {
                'panda1_joint1': np.deg2rad(150), 'panda1_joint2': np.deg2rad(150),
                'panda1_joint3': np.deg2rad(150), 'panda1_joint4': np.deg2rad(150),
                'panda1_joint5': np.deg2rad(180), 'panda1_joint6': np.deg2rad(180), 'panda1_joint7': np.deg2rad(180),
                'indy0_joint0': np.deg2rad(150), 'indy0_joint1': np.deg2rad(150), 'indy0_joint2': np.deg2rad(150),
                'indy0_joint3': np.deg2rad(180), 'indy0_joint4': np.deg2rad(180), 'indy0_joint5': np.deg2rad(180),
            }.items()}
        )
    ZERO_JOINT_POSE = np.array([0, 0, -np.pi / 2, 0, -np.pi / 2, 0,
                                0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, 0])
    refine_meshes()
    xcustom.start_rviz()

from pkg.constraint_graph import *
from pkg.utils.plot_utils import *

# from threading import Thread, Lock

PROC_MODE = True
rospy.init_node('task_planner', anonymous=True)

gtimer = GlobalTimer.instance()
gtimer.reset()
graph = ConstraintGraph(urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES, urdf_content=urdf_content)
col_items_dict = graph.set_fixed_geometry_items(
    get_geometry_items_dict(graph.urdf_content, color=(0,1,0,0.5), display=True, collision=True,
                             exclude_link=["panda1_link7"]))

gtimer.tic("set_scene")
if current_scenario == Scenario.custom_robots:
    collision = True
    graph.add_geometry_items("world",
                              [
                                  GeoMesh(uri="package://my_mesh/meshes/stl/AirPick_cup_ctd.stl",
                                          BLH=(0.01, 0.01, 0.01), scale=(1e-3, 1e-3, 1e-3), name="gripper1",
                                          link_name="indy0_tcp",
                                          urdf_content=graph.urdf_content, color=(0.1, 0.1, 0.1, 1), collision=False),
                                  GeoBox((0.5, -0.2, 0.050), (0.05, 0.05, 0.05), name="box1", link_name="world",
                                         urdf_content=graph.urdf_content, color=(0.3, 0.3, 0.8, 1),
                                         collision=collision),
                                  GeoBox((0, 0, -0.005), (3, 3, 0.01), name="floor", link_name="world",
                                         urdf_content=graph.urdf_content, color=(0.6, 0.6, 0.6, 1),
                                         collision=collision),
                                  GeoBox((0.7, 0.0, 0.2), (0.7, 0.05, 0.4), name="wall", link_name="world",
                                         urdf_content=graph.urdf_content, color=(0.4, 0.3, 0.0, 1),
                                         collision=collision),
                                  GeoBox((0.4, 0.4, 0.15), (0.15, 0.15, 0.3), name="stepper", link_name="world",
                                         urdf_content=graph.urdf_content, color=(0.4, 0.3, 0.0, 1),
                                         collision=collision),
                                  GeoBox((0.4, 0.4, 0.3), (0.1, 0.1, 1e-3), name="goal_disp", link_name="world",
                                         urdf_content=graph.urdf_content, color=(0.8, 0.0, 0.0, 1), collision=False)])

    graph.register_binder(name='grip1', _type=VacuumTool, point=[0, -2.5e-2, 11e-2], link_name="panda1_hand",
                          direction=[0, 1, 0])
    graph.register_binder(name='vac2', _type=VacuumTool, point=[0, 0, 5e-2], link_name="indy0_tcp", direction=[0, 0, 1])
    graph.register_binder(name='floor', _type=PlacePlane, direction=[0, 0, 1])
    graph.register_binder(name='goal', _type=PlaceFrame, point=(0.4, 0.4, 0.3 + 5e-4), link_name="world",
                          orientation=[0, 0, 0])

    graph.register_object('box1', _type=BoxAction, binding=("bottom_p", "floor"), hexahedral=True)

    graph.build_graph()
gtimer.toc("set_scene")

gtimer.tic("set_sim")
graph.set_planner(nWSR=50, regularization_factor= 1e-1)
gtimer.toc("set_sim")
# graph.show_pose(ZERO_JOINT_POSE, execute=True)

gtimer.reset()
if current_scenario == Scenario.custom_robots:
    graph.search_graph(
        initial_state = State((('box1','bottom_p','floor'),),
                              {'box1': SE3(np.identity(3), [0.5,-0.3,0.05])}, ZERO_JOINT_POSE),
        goal_state = State((('box1','front_f','goal'),), None, None),
        tree_margin = 4, depth_margin = 2, joint_motion_num=10,
        terminate_on_first = True, N_search = 100, N_loop=1000,
        display=False, dt_vis=1e-3, verbose = True, print_expression=False,
        **dict(N=300, dt=0.025, vel_conv=1e-2, err_conv=1e-3, N_step=300))