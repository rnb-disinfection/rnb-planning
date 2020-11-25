#!/usr/bin/env python
# coding: utf-8

# In[1]:


from pkg.planner.moveit.moveit_py import init_planner, add_object, clear_objects, plan, ObjectType


# In[2]:


from pkg.utils.utils import read_file
import numpy as np


# In[3]:


urdf_string = read_file('./moveit_plan_compact/test_assets/custom_robots.urdf')
srdf_string = read_file('./moveit_plan_compact/test_assets/custom_robots.srdf')


# In[4]:


joint_names, joint_num = init_planner(urdf_string, srdf_string)


# In[5]:


add_object(name="box", type=ObjectType.BOX, link_name="base_link", pose=[-0.3,-0.2,0.0,0,0,0,1], dims=[0.1,0.1,0.1])


# In[6]:


traj, succ = plan(group_name='indy0', tool_link="indy0_tcp", goal_pose=[-0.3, -0.2, 0.4, 0, 0, 0, 1], goal_link="base_link", 
                  init_joints=[0, 0, -np.pi / 2, 0, -np.pi / 2, 0, 0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 2], 
                  joint_num=joint_num, timeout=0.1)
succ


# In[7]:


clear_objects()
add_object(name="box", type=ObjectType.BOX, link_name="base_link", pose=[-0.3,-0.2,0.4,0,0,0,1], dims=[0.1,0.1,0.1])


# In[8]:


traj, succ = plan(group_name='indy0', tool_link="indy0_tcp", goal_pose=[-0.3, -0.2, 0.4, 0, 0, 0, 1], goal_link="base_link", 
                  init_joints=[0, 0, -np.pi / 2, 0, -np.pi / 2, 0, 0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 2], 
                  joint_num=joint_num, timeout=0.1)
succ


# In[16]:


clear_objects()


# In[17]:


traj, succ = plan(group_name='indy0', tool_link="indy0_tcp", goal_pose=[-0.3, -0.2, 0.4, 0, 0, 0, 1], goal_link="base_link", 
                  init_joints=[0, 0, -np.pi / 2, 0, -np.pi / 2, 0, 0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 2], 
                  joint_num=joint_num, timeout=0.1)
succ


# In[ ]:





# In[ ]:




