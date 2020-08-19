from __future__ import print_function
import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import *
    

# define distances
def make_distance_bound_constraint(ctem1, ctem2, lower=0):
    return """
Constraint{{
context=ctx,
    name="{constraint_name}",
    expr = distance_between({T1},{ctem1},{ctem1radius},margin,{T2},{ctem2},{ctem2radius},margin),
    target_lower = {lower},
    priority = 0
}}""".format(
        constraint_name=ctem1.name+"_"+ctem2.name,
        T1=ctem1.get_tf_name(), ctem1=ctem1.name, ctem1radius=ctem1.get_radius(),
        T2=ctem2.get_tf_name(),ctem2=ctem2.name, ctem2radius=ctem2.get_radius(),
        lower=lower
    )

def make_point_pair_constraint(obj1, obj2, varname, constraint_name, make_error=True, point1=None, point2=None):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{varname}))".format(varname=varname)
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    return """
local {varname}1 = {repre1}
local {varname}2 = {repre2}""".format(
    varname=varname, repre1=obj1.get_representation(point1), repre2=obj2.get_representation(point2)) + \
"""
dist_{varname} = distance_between({T1},{varname}1,{ctem1radius},margin,{T2},{varname}2,{ctem2radius},margin)
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = dist_{varname},
    priority = 2,
    K        = K
}}""".format(
        constraint_name=constraint_name,
        T1=obj1.get_tf_name(), ctem1radius=0,
        T2=obj2.get_tf_name(), ctem2radius=0,
        varname=varname
    ) + error_statement

def make_dir_constraint(pointer1, pointer2, name, constraint_name, make_error=True):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(angle_{name})".format(name=name)
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    return """
vec1 = vector{vec1}
vec2 = vector{vec2}
angle_{name} = angle_between_vectors(vec1,rotation(inv({T1})*{T2})*vec2)""".format(
    T1=pointer1.object.get_tf_name(),
    T2=pointer2.object.get_tf_name(),
    vec1=tuple(pointer1.direction), vec2=tuple(pointer2.direction), name=name) + \
"""
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = angle_{name},
    priority = 2,
    K        = K
}}""".format(
        constraint_name=constraint_name,
        name=name
    ) + error_statement

def make_orientation_constraint(framer1, framer2, name, constraint_name, make_error=True):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(orientation_{name})".format(name=name)
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    R1=Rotation.from_rotvec(framer1.orientation).as_dcm()
    R2=Rotation.from_rotvec(framer2.orientation).as_dcm()
    vec11 = tuple(np.dot(R1, (1,0,0)))
    vec12 = tuple(np.dot(R1, (0,0,1)))
    vec21 = tuple(np.dot(R2, (1,0,0)))
    vec22 = tuple(np.dot(R2, (0,0,1)))
    return """
vec11 = vector{vec11}
vec12 = vector{vec12}
vec21 = vector{vec21}
vec22 = vector{vec22}
tf_{name} = rotation(inv({T1})*{T2})
angle1_{name} = angle_between_vectors(vec11,tf_{name}*vec21)
angle2_{name} = angle_between_vectors(vec12,tf_{name}*vec22)
orientation_{name} = angle1_{name} + angle2_{name}
""".format(
        T1=framer1.object.get_tf_name(),vec11=vec11, vec12=vec12,
        T2=framer2.object.get_tf_name(),vec21=vec21, vec22=vec22,
        name=name) + \
    """
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = orientation_{name},
    priority = 2,
    K        = K
}}""".format(
        constraint_name=constraint_name,
        name=name
    ) + error_statement

def make_directed_point_constraint(pointer1, pointer2, name, make_error=True, point1=None, point2=None):
    error_statement = ""
    constraint_name_point = "point_pair_{name}".format(name=name)
    constraint_name_dir = "dir_pair_{name}".format(name=name)
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{constraint_name_point})+abs(angle_{constraint_name_dir})".format(
            constraint_name_point=constraint_name_point, constraint_name_dir=constraint_name_dir)
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    pair_constraint = make_point_pair_constraint(pointer1.object, pointer2.object, constraint_name_point, constraint_name_point, 
                                                 make_error=False, point1=point1, point2=point2)
    dir_constraint = make_dir_constraint(pointer1, pointer2, name=constraint_name_dir, constraint_name=constraint_name_dir, make_error=False)
    return pair_constraint + "\n" + dir_constraint + error_statement

def make_oriented_point_constraint(framer1, framer2, name, make_error=True, point1=None, point2=None):
    error_statement = ""
    constraint_name_point = "point_pair_{name}".format(name=name)
    constraint_name_ori = "ori_pair_{name}".format(name=name)
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{constraint_name_point})+abs(orientation_{constraint_name_ori})".format(
            constraint_name_point=constraint_name_point, constraint_name_ori=constraint_name_ori)
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    pair_constraint = make_point_pair_constraint(framer1.object, framer2.object, constraint_name_point, constraint_name_point, make_error=False, 
                                                 point1=point1, point2=point2)
    ori_constraint = make_orientation_constraint(framer1, framer2, name=constraint_name_ori, constraint_name=constraint_name_ori, make_error=False)
    return pair_constraint + "\n" + ori_constraint + error_statement
        
def make_collision_constraints(collision_items):
    constraint_text = "\n"
    for ctem1 in collision_items:
        for ctem2 in collision_items[collision_items.index(ctem1)+1:]:
            if ctem2.link_name in ctem1.adjacent_links or ctem1.link_name in ctem2.adjacent_links:
                pass
            else:
                constraint_text += make_distance_bound_constraint(ctem1, ctem2)
    return constraint_text


def make_joint_constraints(joint_names, make_error=True):
    joint_constraints = ""
    error_statement = ""
    for i in range(len(joint_names)):
        joint_constraints += """
target_{joint_name} = ctx:createInputChannelScalar("target_{joint_name}",0.0)
error_{joint_name} = robot_jval[{index}]-target_{joint_name}
Constraint{{
    context=ctx,
    name="constraint_{joint_name}",
    expr=error_{joint_name},
    priority    = 2,
    K           = K
}}
            """.format(joint_name=joint_names[i], index=i+1)
        if make_error:
            error_statement += 'abs(error_{joint_name})+'.format(joint_name=joint_names[i])
    if make_error:
        error_val = "\nerror_target = error_target + "+error_statement[:-1]
        error_statement = error_val+'\n ctx:setOutputExpression("error",error_target)'
    return joint_constraints + error_statement
