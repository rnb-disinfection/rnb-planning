#!/usr/bin/env python
# coding: utf-8

# # Prepare robots first  
# * run panda repeater on controller pc  
# 
# ```
# ssh panda@192.168.0.172
# roslaunch panda_ros_repeater joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=false
# 
# ```
# 
# * Keep indy connected to conty to bypass conty-connection delay bug

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
put_point_dict = graph.register_object_gen(objectPose_dict_mv, BINDER_DICT, OBJECT_DICT, link_name="base_link")


# In[13]:


graph.set_rviz()


# In[ ]:





# ## Register object binders

# In[14]:


register_hexahedral_binder(graph, object_name='box1', _type=PlacePlane)
register_hexahedral_binder(graph, object_name='box2', _type=PlacePlane)




