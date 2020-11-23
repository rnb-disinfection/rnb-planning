#!/usr/bin/env python
# coding: utf-8

# # Run panda repeater first  
# ```
# ssh panda@192.168.0.172
# roslaunch panda_ros_repeater joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=false
# ```

# In[ ]:





# # initialize notebook

# In[1]:


from __future__ import print_function
from IPython.core.display import display, HTML
display(HTML("<style>.container { width:90% !important; } </style>"))
import matplotlib.pyplot as plt


# In[ ]:





# # Initialize constants

# In[2]:


from pkg.marker_config import *
from pkg.constraint_graph import *
from pkg.constraint.constraint_action import *
from pkg.constraint.constraint_object import *
from pkg.constants import *
from pkg.utils.plot_utils import *
from pkg.utils.utils import *
from pkg.environment_builder import *
from pkg.ui.ui_broker import *
from pkg.controller.combined_robot import *

gtimer = GlobalTimer.instance()
gtimer.reset()


# In[ ]:





# In[3]:


crob = CombinedRobot(connection_list=(False, False))


# In[ ]:





# # initialize graph & ui

# In[4]:


if "cam" not in locals():
    cam = StereoCamera.instance()

# set urdf
xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, XYZ_RPY_ROBOTS_DEFAULT, crob.joint_names)

# set graph
if "graph" in locals():
    graph.clear_markers()
    graph.clear_highlight()
    graph.ghnd.clear()
    graph.__init__(urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES, 
                   urdf_content=urdf_content, robots_on_scene=crob.get_dict(), )
    add_geometry_items(graph.urdf_content, color=(0, 1, 0, 0.3), display=True, collision=True,
                       exclude_link=["panda1_link7"])
else:
    graph = ConstraintGraph(urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES, 
                            urdf_content=urdf_content, combined_robot=crob)
    graph.set_camera(cam)
    graph.set_cam_robot_collision()
    graph.set_rviz()
    
    # start UI
    ui_broker = UIBroker(graph)
    ui_broker.start_server()
    
    # set rviz
    graph.set_rviz(crob.home_pose)
    ui_broker.set_tables()


# In[ ]:





# # Calibrate camera

# In[5]:


cam.calibrate()
graph.set_cam_robot_collision()
graph.set_rviz()


# In[ ]:





# # Update Robots

# In[6]:


# # btn: detect robots
# crob.detect_robots(cam)

# # btn: apply
# xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, crob.xyz_rpy_robots, crob.joint_names)


# graph.clear_markers()
# graph.clear_highlight()
# graph.ghnd.clear()
# timer.sleep(1)
# graph.__init__(urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
#                urdf_content=urdf_content, combined_robot=crob)
# add_geometry_items(graph.urdf_content, color=(0, 1, 0, 0.3), display=True, collision=True,
#                    exclude_link=["panda1_link7"])
# graph.set_cam_robot_collision()
# graph.set_rviz()


# In[ ]:





# # Detect environment

# In[7]:


env_gen_dict, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image = cam.detect_environment()
add_objects_gen(graph, env_gen_dict)
graph.set_rviz()


# In[ ]:





# # Reset robot connection

# In[8]:


graph.combined_robot.reset_connection([False, False])


# In[ ]:





# # Go home

# In[9]:


if all(graph.combined_robot.connection_list):
    graph.combined_robot.joint_make_sure(graph.combined_robot.home_pose)


# In[ ]:





# ## Register binders

# In[10]:


graph.register_binder(name='grip1', _type=Gripper2Tool, point=[0,0,0.112], rpy=[-np.pi/2,0,0], link_name="panda1_hand")
graph.register_binder(name='grip0', _type=Gripper2Tool, point=[0,0,0.14], rpy=[-np.pi/2,0,0], link_name="indy0_tcp")
graph.register_binder(name='floor', _type=PlacePlane)


# In[ ]:





# ## detect movable

# In[11]:


BINDER_DICT = {'goal_bd': dict(_type=PlacePlane, object_name="goal", point=[0,0,0.02], rpy=[0,0,0])}
OBJECT_DICT = {'box1': dict(_type=BoxAction, hexahedral=True),
               'box2': dict(_type=BoxAction, hexahedral=True)}


# In[12]:


objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv = detect_objects(graph.cam.aruco_map, graph.cam.dictionary, graph.cam.ref_tuple)
put_point_dict = graph.register_object_gen(objectPose_dict_mv, BINDER_DICT, OBJECT_DICT, link_name="world")


# In[13]:


graph.set_rviz()


# In[ ]:





# ## Register object binders

# In[14]:


register_hexahedral_binder(graph, object_name='box1', _type=PlacePlane)
register_hexahedral_binder(graph, object_name='box2', _type=PlacePlane)


# In[15]:


1


# In[ ]:





# # Tesseract

# In[17]:


import tesseract
import os
import re
import numpy as np
import time
from pkg.utils.utils import *
from pkg.global_config import *
from pkg.utils.utils import *

import rospkg
rospack = rospkg.RosPack()

class RosResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(RosResourceLocator,self).__init__()

    def locateResource(self, url):
        fname = None
        if "package://" in url:
            url_split = url[10:].split("/")
            pkg_name = url_split[0]
            pkg_path = rospack.get_path(pkg_name)
            fname = os.path.join(pkg_path, *url_split[1:])
            
            with open(fname,'rb') as f:
                resource_bytes = f.read()
        else:
            raise(RuntimeError("unknown resource"))
            return None

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource

# base_names=['indy0_link0', 'panda1_link0'], link_names=['indy0_tcp', 'panda1_hand']
def load_custom_robot():
    with open(os.path.join(PROJ_DIR, "robots", "custom_robots.urdf"),'r') as f:
        robot_urdf = f.read()

    with open(os.path.join(PROJ_DIR, "robots", "custom_robots.srdf"),'r') as f:
        robot_srdf = f.read()

    tes = tesseract.Tesseract()

    tes.init(robot_urdf, robot_srdf, RosResourceLocator())
    return tes

@record_time
def plan_manipulation(tes, robot_name, end_link, Tbe, base_link="world"):

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

    start_pos_terminfo.targets = np.array([0.0]*6, dtype=np.float64)

    end_pos_terminfo = tesseract.CartPoseTermInfo()
    xyz = Tbe[:3,3]
    wxyz = Rotation.from_dcm(Tbe[:3,:3]).as_quat()[[3,0,1,2]]
    jsonval =     """
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
    end_pos_terminfo.first_step = pci.basic_info.n_steps-1
    end_pos_terminfo.last_step = pci.basic_info.n_steps-1
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

def plan_robot(tes, robot_name, binder, action_point, joint_dict, from_link='world'):
    Tap = ap.get_tf_handle(joint_dict, from_link=from_link)
    Tbe = np.matmul(Tap, SE3_inv(binder.Toff_lh))
    return plan_manipulation(tes, robot_name, binder.object.link_name, Tbe)


# In[ ]:





# In[18]:


robot_name = 'indy0'
binder = graph.binder_dict['grip0']
bx2 = graph.object_dict['box2']
ap = bx2.action_points_dict['top_g']


# In[19]:


tes = load_custom_robot()


# In[20]:


env = tes.getEnvironment()
link_names = env.getLinkNames()
for lname in link_names:
    link = env.getLink(lname)
    for collision in link.collision:
        geometry = collision.geometry
        if geometry.getType() == tesseract.CYLINDER:
            glen = geometry.getLength()
            grad = geometry.getRadius()
            collision.geometry = tesseract.Capsule(grad,glen)
            
for gtem in graph.ghnd:
    if gtem.collision:
        if robot_name in gtem.name:
            continue
        if gtem.gtype == GEOTYPE.SPHERE:
            colgeo = tesseract.Sphere(gtem.radius)
        elif gtem.gtype == GEOTYPE.SEGMENT:
            colgeo = tesseract.Capsule(gtem.radius, gtem.length)
        elif gtem.gtype == GEOTYPE.BOX:
            colgeo = tesseract.Box(*gtem.dims)
        elif gtem.gtype == GEOTYPE.MESH:
            raise(RuntimeError("Mesh collision is not implemented"))
        elif gtem.gtype == GEOTYPE.ARROW:
            raise(RuntimeError("Arrow collision is not implemented"))
        else:
            raise(RuntimeError("Unknown geometry"))

        coltem = tesseract.Collision()
        coltem.geometry = colgeo
        coltem.origin = gtem.Toff.astype(np.float64)
        link = env.getLink(gtem.link_name)
        link.collision.push_back(coltem)
        vistem = tesseract.Visual()
        vistem.geometry = colgeo.clone()
        vistem.origin = coltem.origin


# In[22]:


tes = tesseract.Tesseract()
tes.init(env.getSceneGraph())


# In[ ]:


traj = plan_robot(tes, robot_name, binder, ap, graph.combined_robot.home_dict)


# In[21]:


Qall = []
for jt in traj:
    Q = graph.combined_robot.home_pose.copy()
    Q[graph.combined_robot.idx_dict[robot_name]] = jt
    Qall.append(Q)
graph.show_motion(Qall, period=0.05)


# In[22]:


from tesseract_viewer import TesseractViewer
viewer = TesseractViewer()


# In[23]:


viewer.update_environment(env, [0,0,0])
joint_names = env.getActiveJointNames()
viewer.update_joint_positions(joint_names, env.getCurrentJointValues())


# In[24]:


viewer.start_serve_background()


# In[25]:


1


# In[26]:


wlink = env.getLink('world')


# In[27]:


len(wlink.collision)


# In[28]:


wlink.collision[0]


# In[ ]:





# # Set Planner

# In[21]:


eplan = etasl_planner(joint_names = graph.joint_names, link_names = graph.link_names, urdf_path = graph.urdf_path)
graph.set_planner(eplan)


# In[ ]:





# # Set Sampler

# In[17]:


from pkg.sampler.object_a_star import *
sampler = ObjectAstarSampler(graph)
graph.set_sampler(sampler)


# In[ ]:





# # Search Graph (HandleAstar)

# In[18]:


gtimer.reset()


# In[19]:


sampler.build_graph()
print(graph.gtimer)


# In[20]:


OBJECT_DICT = {k:dict(_type=v.__class__) for k,v in graph.object_dict.items()}
objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv =     detect_objects(graph.cam.aruco_map, graph.cam.dictionary)
xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(
    objectPose_dict_mv, graph.cam.aruco_map, OBJECT_DICT, graph.cam.ref_tuple)
update_geometries(objectPose_dict_mv.keys(), objectPose_dict_mv, graph.cam.ref_tuple[1])
graph.set_rviz()


# In[21]:


dt_sim = 0.04
T_step = 10
N_fullstep = int(T_step / dt_sim)
gtimer.reset()
initial_state = State(tuple([(oname, put_point_dict[oname],'floor') for oname in graph.object_list]), 
                      {oname: graph.object_dict[oname].object.Toff for oname in graph.object_list}, 
                      graph.get_real_robot_pose(), graph)
binding_dict = match_point_binder(graph, initial_state, objectPose_dict_mv)
initial_state = State(tuple([(oname, put_point_dict[oname],binding_dict[oname]) for oname in graph.object_list]), 
                      {oname: graph.object_dict[oname].object.Toff for oname in graph.object_list}, 
                      graph.get_real_robot_pose(), graph)
graph.set_object_state(initial_state)
graph.show_pose(graph.get_real_robot_pose())

goal_nodes_1 = get_goal_nodes(initial_state.node, "box1", "goal_bd")
goal_nodes = []
for gnode in goal_nodes_1:
    goal_nodes += get_goal_nodes(gnode, "box2", "floor")
# goal_nodes = goal_nodes[3:4]

sampler.search_graph(
    initial_state = initial_state, goal_nodes = goal_nodes,
    tree_margin = 2, depth_margin = 2, 
    terminate_on_first = True, N_search = 100, N_loop=1000, multiprocess=True, sample_num=30,
    display=True, dt_vis=dt_sim/4, verbose = True, print_expression=False, error_skip=0, traj_count=DEFAULT_TRAJ_COUNT,
    **dict(N=N_fullstep, dt=dt_sim, vel_conv=0.5e-2, err_conv=1e-3))

schedule_dict = sampler.find_schedules()
schedule_sorted = sampler.sort_schedule(schedule_dict)


# In[22]:


print(gtimer)
sampler.quiver_snodes()


# In[23]:


1


# # Replay schedule

# In[24]:


schedule_dict = sampler.find_schedules()
schedule_sorted = sampler.sort_schedule(schedule_dict)
N_fullstep = 500
dt_sim = 0.04
# dt_sim = 0.04
for schedule in schedule_sorted:
    print(schedule)
for schedule, i_s in zip(schedule_sorted, range(len(schedule_sorted))):
    traj, end_state, error, success = graph.test_transition(
        sampler.snode_dict[0].state, sampler.snode_dict[0].state, 
        N=10, dt=dt_sim, vel_conv=1e-2, err_conv=5e-4, print_expression=False)
    timer.sleep(0.1)
#     try:
    e = graph.replay(schedule, N=N_fullstep, dt=dt_sim, 
                     vel_conv=0.5e-2, err_conv=1e-3, error_skip=0)
#     except Exception as e:
#         print(e)


# In[ ]:





# # Simulate traj online

# In[28]:


snode_schedule = graph.sampler.find_best_schedule(schedule_sorted)
eplan.update(graph)

with DynamicDetector(eplan.online_names, graph.cam.aruco_map, graph.cam.dictionary, graph.cam.rs_config, graph.cam.T_c12, graph.cam.ref_tuple[1]) as dynamic_detector,         RvizPublisher(graph, eplan.online_names) as rviz_pub:
    e_sim = graph.execute_schedule_online(snode_schedule, eplan, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                  vel_conv=0, err_conv=1e-3, T_step = 100, on_rviz=True, 
                                  dynamic_detector=dynamic_detector, rviz_pub=rviz_pub, obs_K="40")


# In[ ]:





# # Execute traj online

# In[29]:


graph.combined_robot.reset_connection([True, True])


# In[30]:


snode_schedule = graph.sampler.find_best_schedule(schedule_sorted)
eplan.update(graph)

with DynamicDetector(eplan.online_names, graph.cam.aruco_map, graph.cam.dictionary, graph.cam.rs_config, graph.cam.T_c12, graph.cam.ref_tuple[1]) as dynamic_detector,         RvizPublisher(graph, eplan.online_names) as rviz_pub:
    e_sim = graph.execute_schedule_online(snode_schedule, eplan, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                  vel_conv=0, err_conv=1e-3, T_step = 100, on_rviz=False, 
                                  dynamic_detector=dynamic_detector, rviz_pub=rviz_pub, obs_K="40")


# In[31]:


print(gtimer)


# In[ ]:





# In[40]:


plt.plot(gtimer.timelist_dict["move_wait"])
plt.axis([0,15000,0,50])


# In[41]:


plt.plot(gtimer.timelist_dict["send-robot-0"])
plt.axis([0,6000,0,10])


# In[47]:


plt.plot(gtimer.timelist_dict["count-robot-0"])
# plt.axis([0,6000,0,10])


# In[43]:


plt.plot(gtimer.timelist_dict["send-robot-1"])
plt.axis([0,6000,0,10])


# In[46]:


plt.plot(gtimer.timelist_dict["count-robot-1"])
# plt.axis([0,6000,0,20])


# In[ ]:





# In[ ]:





# In[ ]:





# # onestep

# In[36]:


loop_process = graph
loop_process.stop_now = False
while not loop_process.stop_now:
    print("wait for button input")
    graph.indy.connect_and(graph.indy.wait_di, 16)
    if loop_process.stop_now:
        break
    kn_config, rs_config, T_c12 = calibrate_stereo(aruco_map, dictionary)
    objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv =         detect_objects(MOVABLE_GENERATORS, aruco_map, dictionary)
    objectPose_dict_mv.update({'floor': objectPose_dict['floor']})
    xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(
        objectPose_dict_mv, MOVABLE_GENERATORS, OBJECT_DICT, ("floor", objectPose_dict["floor"]))
    update_geometries(MOVABLE_GENERATORS.keys(), objectPose_dict_mv)
    graph.show_pose(ZERO_JOINT_POSE)

    dt_sim = 0.04
    T_step = 10
    N_fullstep = int(T_step / dt_sim)
    gtimer.reset()
    initial_state = State(tuple([(oname, put_point_dict[oname],'floor') for oname in graph.object_list]), 
                          {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list}, 
                          (graph.get_real_robot_pose() if graph.connect_indy and graph.connect_panda 
                           else ZERO_JOINT_POSE))
    binding_dict = match_point_binder(graph, initial_state, objectPose_dict_mv)
    initial_state = State(tuple([(oname, put_point_dict[oname],binding_dict[oname]) for oname in graph.object_list]), 
                          {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list}, 
                          (graph.get_real_robot_pose() if graph.connect_indy and graph.connect_panda 
                           else ZERO_JOINT_POSE))
    graph.set_object_state(initial_state)
    graph.show_pose(ZERO_JOINT_POSE)

    goal_nodes_1 = get_goal_nodes(initial_state.node, "box1", "goal_bd")
    goal_nodes = []
    for gnode in goal_nodes_1:
        goal_nodes += get_goal_nodes(gnode, "box2", "floor")
    # goal_nodes = goal_nodes[3:4]

    graph.search_graph_mp(
        initial_state = initial_state, goal_nodes = goal_nodes, swept_volume_test_jmotion=False,
        tree_margin = 2, depth_margin = 2, joint_motion_num=10, 
        terminate_on_first = True, N_search = 100, N_loop=1000,
        display=False, dt_vis=dt_sim/40, verbose = True, print_expression=False, error_skip=0,
        **dict(N=N_fullstep, dt=dt_sim, vel_conv=1e-3, err_conv=1e-3, N_step=N_fullstep))

    schedule_dict = graph.find_schedules()
    schedule_sorted = graph.sort_schedule(schedule_dict)
    print(gtimer)
    schedule_dict = graph.find_schedules()
    schedule_sorted = graph.sort_schedule(schedule_dict)

    schedule = schedule_sorted[0]
    state_schedule = graph.idxSchedule2stateScedule(schedule, ZERO_JOINT_POSE)
    obs_names = ["box3"]

    with DynamicDetector(obs_names, aruco_map, dictionary, rs_config, T_c12, objectPose_dict['floor']) as dynamic_detector,             RvizPublisher(graph, obs_names) as rviz_pub:
        graph.execute_schedule_online(state_schedule, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                      vel_conv=1e-2, err_conv=1e-3, T_step = 100, on_rviz=False, 
                                      obs_names=obs_names, dynamic_detector=dynamic_detector, rviz_pub=rviz_pub)
print("===========================================")
print("=================KILLED====================")
print("===========================================")


# In[ ]:





# In[ ]:


def loop_process():
    loop_process.stop_now = False
    while not loop_process.stop_now:
        print("wait for button input")
        graph.indy.connect_and(graph.indy.wait_di, 16)
        if loop_process.stop_now:
            break
        kn_config, rs_config, T_c12 = calibrate_stereo(aruco_map, dictionary)
        objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv =             detect_objects(MOVABLE_GENERATORS, aruco_map, dictionary)
        objectPose_dict_mv.update({'floor': objectPose_dict['floor']})
        xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(
            objectPose_dict_mv, MOVABLE_GENERATORS, OBJECT_DICT, ("floor", objectPose_dict["floor"]))
        update_geometries(MOVABLE_GENERATORS.keys(), objectPose_dict_mv)
        graph.show_pose(ZERO_JOINT_POSE)
        
        dt_sim = 0.04
        T_step = 10
        N_fullstep = int(T_step / dt_sim)
        gtimer.reset()
        initial_state = State(tuple([(oname, put_point_dict[oname],'floor') for oname in graph.object_list]), 
                              {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list}, 
                              (graph.get_real_robot_pose() if graph.connect_indy and graph.connect_panda 
                               else ZERO_JOINT_POSE))
        binding_dict = match_point_binder(graph, initial_state, objectPose_dict_mv)
        initial_state = State(tuple([(oname, put_point_dict[oname],binding_dict[oname]) for oname in graph.object_list]), 
                              {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list}, 
                              (graph.get_real_robot_pose() if graph.connect_indy and graph.connect_panda 
                               else ZERO_JOINT_POSE))
        graph.set_object_state(initial_state)
        graph.show_pose(ZERO_JOINT_POSE)

        goal_nodes_1 = get_goal_nodes(initial_state.node, "box1", "goal_bd")
        goal_nodes = []
        for gnode in goal_nodes_1:
            goal_nodes += get_goal_nodes(gnode, "box2", "floor")
        # goal_nodes = goal_nodes[3:4]

        graph.search_graph_mp(
            initial_state = initial_state, goal_nodes = goal_nodes, swept_volume_test_jmotion=False,
            tree_margin = 2, depth_margin = 2, joint_motion_num=10, 
            terminate_on_first = True, N_search = 100, N_loop=1000,
            display=False, dt_vis=dt_sim/40, verbose = True, print_expression=False, error_skip=0,
            **dict(N=N_fullstep, dt=dt_sim, vel_conv=1e-3, err_conv=1e-3, N_step=N_fullstep))

        schedule_dict = graph.find_schedules()
        schedule_sorted = graph.sort_schedule(schedule_dict)
        print(gtimer)
        schedule_dict = graph.find_schedules()
        schedule_sorted = graph.sort_schedule(schedule_dict)

        schedule = schedule_sorted[0]
        state_schedule = graph.idxSchedule2stateScedule(schedule, ZERO_JOINT_POSE)
        obs_names = ["box3"]

        with DynamicDetector(obs_names, aruco_map, dictionary, rs_config, T_c12, objectPose_dict['floor']) as dynamic_detector,                 RvizPublisher(graph, obs_names) as rviz_pub:
            graph.execute_schedule_online(state_schedule, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                          vel_conv=0.2e-2, err_conv=1e-3, T_step = 100, on_rviz=False, 
                                          obs_names=obs_names, dynamic_detector=dynamic_detector, rviz_pub=rviz_pub)
    print("===========================================")
    print("=================KILLED====================")
    print("===========================================")
t_loop = Thread(target=loop_process)
t_loop.start()


# In[ ]:


loop_process.stop_now=True


# In[ ]:


t_loop.is_alive()


# In[ ]:





# In[ ]:


plt.figure(figsize=(20,10))
plt.subplot(1,2,1)
ax = plt.plot(qcount_indy_list)
ax = plt.plot(qcount_panda_list)
plt.subplot(1,2,2)
ax = plt.plot(looptime_list)
print(gtimer)


# In[ ]:





# In[ ]:





# In[ ]:





# # Item handling scripts

# ## show env image

# In[7]:


# screen_size = (1080,1920)
# kn_image_out = draw_objects(color_image, aruco_map, objectPose_dict, corner_dict, *kn_config, axis_len=0.1)
# rs_image_out = draw_objects(rs_image, aruco_map, {k:v for k,v in rs_objectPose_dict.items() if k != 'wall'}, rs_corner_dict, *rs_config, axis_len=0.1)
# kn_image_out_res = cv2.resize(kn_image_out, (rs_image_out.shape[1], rs_image_out.shape[0]))
# image_out = np.concatenate([kn_image_out_res, rs_image_out], axis=1)
# ratio = np.min(np.array(screen_size,dtype=np.float)/np.array(image_out.shape[:2],dtype=np.float))
# image_out_res = cv2.resize(image_out, (int(image_out.shape[1]*ratio), int(image_out.shape[0]*ratio)))
# cv2.imshow("test", image_out_res)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# In[8]:


kn_image_out = draw_objects(color_image, aruco_map, objectPose_dict, corner_dict, *kn_config, axis_len=0.1)
rs_image_out = draw_objects(rs_image, aruco_map, {k:v for k,v in rs_objectPose_dict.items() if k != 'wall'}, rs_corner_dict, *rs_config, axis_len=0.1)
plt.figure(figsize=(30,10))
plt.subplot(1,2,1)
plt.imshow(kn_image_out[:,:,[2,1,0]])
plt.subplot(1,2,2)
plt.imshow(rs_image_out[:,:,[2,1,0]])


# ## Register binders

# In[11]:


graph.register_binder(name='grip1', _type=Gripper2Tool, point=[0,0,0.112], link_name="panda1_hand", direction=[0,1,0])
graph.register_binder(name='grip0', _type=Gripper2Tool, point=[0,0,0.14], link_name="indy0_tcp", direction=[0,1,0])
graph.register_binder(name='floor', _type=PlacePlane, direction=[0,0,1])


# ## detect movable

# In[12]:


BINDER_DICT = {'goal_bd': dict(_type=PlacePlane, object_name="goal", point=[0,0,0.02], direction=[0,0,1])}
OBJECT_DICT = {'box1': dict(_type=BoxAction, hexahedral=True),
               'box2': dict(_type=BoxAction, hexahedral=True)}


# In[13]:


objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv = detect_objects(aruco_map, dictionary)
put_point_dict = graph.register_object_gen(objectPose_dict_mv, BINDER_DICT, OBJECT_DICT, 
                 ref_tuple=(REF_NAME, objectPose_dict[REF_NAME]), link_name="world")


# # show markers

# In[14]:


plt.figure(figsize=(10,8))
aruco_map_mv.update({'floor': aruco_map['floor']})
color_image_out = draw_objects(color_image, aruco_map_mv, objectPose_dict_mv, corner_dict_mv, *kn_config, axis_len=0.1)
plt.imshow(color_image_out[:,:,[2,1,0]])


# In[ ]:





# # Register object binders

# In[17]:


register_hexahedral_binder(graph, object_name='box1', _type=PlacePlane)
register_hexahedral_binder(graph, object_name='box2', _type=PlacePlane)


# In[ ]:





# # Set ui data

# In[16]:


graph.set_rviz(ZERO_JOINT_POSE)
ui_broker.set_tables()


# In[ ]:





# In[23]:


np.rad2deg(graph.planner.inp[3:-2] - graph.joints.position)


# In[25]:


graph.show_pose(graph.planner.inp[3:-2])


# In[25]:


graph.show_pose(graph.planner.inp[3:-2])


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# # Going back

# In[ ]:


e_POS = e_POS_list[0]
self.panda.move_joint_interpolated(
    e_POS[0, graph.panda_idx], N_step=500, N_div=500)


# In[ ]:





# In[ ]:


e_POS = e_POS_list[0]
if hasattr(graph, 'indy'):
    graph.indy.joint_move_to(np.rad2deg(e_POS[0,graph.indy_idx]))


# In[ ]:


self.reset_panda()


# In[ ]:





# In[ ]:





# In[4]:


'x' in [u'x', u'y']


# In[ ]:





# ## test unavailable binder

# In[50]:


from_state = graph.snode_dict[2].state
to_state = from_state.copy()
print(to_state.node)
print(sorted(graph.binder_dict.keys()))


# In[47]:


to_state.node = (('box1', 'back_p', 'box2_right'), ('box2', 'front_p', 'grip0'))


# In[48]:


traj, new_state, error, succ = graph.test_transition(from_state, to_state, display=True, N=N_fullstep, dt=dt_sim, vel_conv=1e-3, err_conv=1e-3)


# In[ ]:





# # test mesh

# In[ ]:


from pkg.gjk import *


# In[ ]:


Q1 = ZERO_JOINT_POSE
Q2 = ZERO_JOINT_POSE - 0.1

gtimer.reset()

gtimer.tic("svt")
swept_volume_test(Q1, Q2, graph.fixed_collision_items_list+graph.movable_collision_items_list, 
                  graph.joint_names, graph.urdf_content)
gtimer.toc("svt")


# In[ ]:


import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mplot3d

fig = plt.figure(figsize=(15, 5))

idx_col = 205

sub = fig.add_subplot(1, 1, 1, projection="3d")
x, y, z = np.transpose(vtx2_list[idx1_list[idx_col]])
sub.plot(x, y, z, '-o')
x, y, z = np.transpose(vtx2_list[idx2_list[idx_col]])
print(GeometryItem.GLOBAL_GEO_LIST[idx1_list[idx_col]].name)
print(GeometryItem.GLOBAL_GEO_LIST[idx2_list[idx_col]].name)
sub.plot(x, y, z, '-o')
# sub.view_init(0, 90)


# In[ ]:


# np.save("vtx_list", vtx_list)
# np.save("radius_list", radius_list)
# np.save("idx1_list", idx1_list)
# np.save("idx2_list", idx2_list)


# In[ ]:





# # test time

# In[ ]:


gtimer.reset()
found_vec = []
for _ in range(10):
    objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv =         detect_objects(MOVABLE_GENERATORS, aruco_map, dictionary)
    objectPose_dict_mv.update({'floor': objectPose_dict['floor']})
    xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(
        objectPose_dict_mv, MOVABLE_GENERATORS, OBJECT_DICT, ("floor", objectPose_dict["floor"]))
    update_geometries(MOVABLE_GENERATORS.keys(), objectPose_dict_mv)
    graph.show_pose(ZERO_JOINT_POSE)
    
    dt_sim = 0.04
    T_step = 10
    N_fullstep = int(T_step / dt_sim)
    initial_state = State(tuple([(oname, put_point_dict[oname],'floor') for oname in graph.object_list]), 
                          {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list}, 
                          (graph.get_real_robot_pose() if graph.connect_indy and graph.connect_panda 
                           else ZERO_JOINT_POSE))

    goal_nodes = get_goal_nodes(initial_state, "box1", "goal_bd")
    # goal_nodes = goal_nodes[3:4]

    graph.search_graph_mp(
        initial_state = initial_state, goal_nodes = goal_nodes, swept_volume_test_jmotion=True,
        tree_margin = 2, depth_margin = 2, joint_motion_num=20, 
        terminate_on_first = True, N_search = 100, N_loop=1000,
        display=False, dt_vis=dt_sim/40, verbose = True, print_expression=False, error_skip=0,
        **dict(N=N_fullstep, dt=dt_sim, vel_conv=1e-2, err_conv=1e-3, N_step=N_fullstep))

    schedule_dict = graph.find_schedules()
    schedule_sorted = graph.sort_schedule(schedule_dict)
    found_vec.append(len(schedule_sorted)>0)
print(gtimer)


# In[ ]:





# In[ ]:


print("Success rate: {} %".format(np.mean(found_vec)*100))
print(gtimer)


# In[ ]:


# SVT CASE
# Success rate: 100.0 %
# show_pose: 	637.0 ms/30 = 21.22 ms (18.16/27.518)
# search_graph_mp: 	99415.0 ms/30 = 3313.844 ms (1712.408/4521.561)
# init_search: 	57.0 ms/30 = 1.908 ms (1.497/7.065)
# score_graph: 	29.0 ms/240 = 0.122 ms (0.086/0.272)
# reset_valid_node: 	14.0 ms/6030 = 0.002 ms (0.001/0.025)
# check_goal: 	4.0 ms/6177 = 0.001 ms (0.0/0.021)
# get_valid_neighbor: 	6.0 ms/540 = 0.011 ms (0.002/3.62)
# add_node_queue_leafs: 	113.0 ms/30 = 3.767 ms (2.884/5.323)
# find_schedules: 	28.0 ms/30 = 0.948 ms (0.604/1.509)
# sort_schedule: 	0.0 ms/30 = 0.01 ms (0.007/0.022)


# In[ ]:


# NO SVT CASE
# Success rate: 100.0 %
# show_pose: 	609.0 ms/30 = 20.284 ms (17.837/26.163)
# search_graph_mp: 	96204.0 ms/30 = 3206.813 ms (1566.82/4633.293)
# init_search: 	53.0 ms/30 = 1.753 ms (1.495/2.953)
# score_graph: 	28.0 ms/240 = 0.116 ms (0.088/0.312)
# reset_valid_node: 	14.0 ms/6030 = 0.002 ms (0.001/0.022)
# check_goal: 	4.0 ms/6174 = 0.001 ms (0.0/0.015)
# get_valid_neighbor: 	2.0 ms/540 = 0.004 ms (0.002/0.026)
# add_node_queue_leafs: 	109.0 ms/30 = 3.65 ms (2.816/4.925)
# find_schedules: 	28.0 ms/30 = 0.941 ms (0.617/2.113)
# sort_schedule: 	0.0 ms/30 = 0.01 ms (0.007/0.015)


# In[ ]:




