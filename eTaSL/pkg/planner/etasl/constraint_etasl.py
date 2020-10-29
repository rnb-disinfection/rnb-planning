from __future__ import print_function
from ...constraint.constraint_object import *

def get_tf_name(gtem):
    return "{}_tf".format(gtem.name)

def get_link_transformation(link_name):
    return "T_{link_name}".format(link_name=link_name)

def get_tf_representation(gtem):
    orientation_mat = gtem.orientation_mat
    angle_option = ""
    if (3 - np.sum(np.diag(orientation_mat))) > 1e-4:
        zyxrot = Rot2zyx(orientation_mat)
        for i in range(len(zyxrot)):
            if abs(zyxrot[2 - i]) > 1e-4:
                angle_option = "*rotate_{axis}({val})".format(axis="xyz"[i], val=zyxrot[2 - i]) + angle_option
    center = gtem.get_center()
    center_option = ""
    if np.sum(np.abs(center)) > 1e-4:
        for i in range(len(center)):
            if abs(center[i]) > 1e-4:
                center_option += "*translate_{axis}({val})".format(axis="xyz"[i], val=center[i])
    tf_text = "{tf_name} = {Tname}{center_option}{angle_option}".format(
        tf_name=get_tf_name(gtem), Tname=get_link_transformation(gtem.link_name), center_option=center_option, angle_option=angle_option)
    return tf_text

def get_representation(gtem, point=None):
    if gtem.gtype == GEOTYPE.SPHERE:
        if point is None:
            return "MultiSphere({{Vector({},{},{})}},{{ {} }})".format(0, 0, 0, 0) #gtem.radius)
        else:
            raise NotImplementedError
    elif gtem.gtype == GEOTYPE.BOX:
        if point is None:
            return "Box({},{},{})".format(*gtem.dims)
        else:
            return "MultiSphere({{Vector({},{},{})}},{{ {} }})".format(*(tuple(point)+(0,)))
    elif gtem.gtype == GEOTYPE.SEGMENT:
        if point is None:
            return "CapsuleZ({radius},{length})".format(radius=gtem.radius,length=gtem.dims[2])
        else:
            raise NotImplementedError
    elif gtem.gtype == GEOTYPE.MESH:
        return "Box({},{},{})".format(*gtem.dims)
    

# define distances
def make_distance_bound_constraint(ctem1, ctem2, lower=0, K="K"):
    soft = ctem1.soft or ctem2.soft
    if ctem1.K_col is not None or ctem2.K_col is not None:
        K = max(ctem1.K_col, ctem2.K_col)
    return """
Constraint{{
context=ctx,
    name="{constraint_name}",
    expr = distance_between({T1},{ctem1},{ctem1radius},margin,{T2},{ctem2},{ctem2radius},margin),
    target_lower = {lower},
    priority = {priority},
}}""".format(
        constraint_name=ctem1.name+"_"+ctem2.name,
        T1=get_tf_name(ctem1), ctem1=ctem1.name, ctem1radius=ctem1.radius,
        T2=get_tf_name(ctem2),ctem2=ctem2.name, ctem2radius=ctem2.radius,
        lower=lower, priority= (0 if not soft else "2,\n    K = {K}".format(K=K))
    )

def make_point_pair_constraint(obj1, obj2, varname, constraint_name, make_error=True, point1=None, point2=None,
                               K='K', activation=False):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{varname}))".format(varname=varname)
        error_statement = error_val+'\nctx:setOutputExpression("error",error_target)'
    return """
local {varname}1 = {repre1}
local {varname}2 = {repre2}""".format(
    varname=varname, repre1=get_representation(obj1, point1), repre2=get_representation(obj2, point2)) + \
"""
dist_{varname} = distance_between({T1},{varname}1,{ctem1radius},margin,{T2},{varname}2,{ctem2radius},margin)
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = {activation_expr}dist_{varname},
    priority = 2,
    K        = {K}
}}""".format(
        constraint_name=constraint_name,
        T1=get_tf_name(obj1), ctem1radius=0,
        T2=get_tf_name(obj2), ctem2radius=0,
        varname=varname, K=K, activation_expr="constraint_activation*" if activation else ""
    ) + error_statement

def make_dir_constraint(pointer1, pointer2, name, constraint_name, make_error=True, K='K', activation=False):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(angle_{name})".format(name=name)
        error_statement = error_val+'\nctx:setOutputExpression("error",error_target)'
    return """
vec1 = vector{vec1}
vec2 = vector{vec2}
angle_{name} = angle_between_vectors(vec1,rotation(inv({T1})*{T2})*vec2)""".format(
    T1=get_tf_name(pointer1.object),
    T2=get_tf_name(pointer2.object),
    vec1=tuple(pointer1.direction), vec2=tuple(pointer2.direction), name=name) + \
"""
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = {activation_expr}angle_{name},
    priority = 2,
    K        = {K}
}}""".format(
        constraint_name=constraint_name,
        name=name, K=K, activation_expr="constraint_activation*" if activation else ""
    ) + error_statement

def make_orientation_constraint(framer1, framer2, name, constraint_name, make_error=True, K='K', activation=False):
    error_statement = ""
    if make_error:
        error_val = "\nerror_target = error_target + abs(orientation_{name})".format(name=name)
        error_statement = error_val+'\nctx:setOutputExpression("error",error_target)'
    R1=framer1.orientation_mat
    R2=framer2.orientation_mat
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
        T1=get_tf_name(framer1.object),vec11=vec11, vec12=vec12,
        T2=get_tf_name(framer2.object),vec21=vec21, vec22=vec22,
        name=name) + \
    """
Constraint{{
    context=ctx,
    name="{constraint_name}",
    expr = {activation_expr}orientation_{name},
    priority = 2,
    K        = {K}
}}""".format(
        constraint_name=constraint_name,
        name=name, K=K, activation_expr="constraint_activation*" if activation else ""
    ) + error_statement

def make_directed_point_constraint(pointer1, pointer2, name, make_error=True, point1=None, point2=None, activation=False):
    error_statement = ""
    constraint_name_point = "point_pair_{name}".format(name=name)
    constraint_name_dir = "dir_pair_{name}".format(name=name)
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{constraint_name_point})+abs(angle_{constraint_name_dir})".format(
            constraint_name_point=constraint_name_point, constraint_name_dir=constraint_name_dir)
        error_statement = error_val+'\nctx:setOutputExpression("error",error_target)'
    pair_constraint = make_point_pair_constraint(pointer1.object, pointer2.object, constraint_name_point, constraint_name_point, 
                                                 make_error=False, point1=point1, point2=point2, activation=activation)
    dir_constraint = make_dir_constraint(pointer1, pointer2, name=constraint_name_dir, constraint_name=constraint_name_dir,
                                         make_error=False, activation=activation)
    return pair_constraint + "\n" + dir_constraint + error_statement

def make_oriented_point_constraint(framer1, framer2, name, make_error=True, point1=None, point2=None, activation=False):
    error_statement = ""
    constraint_name_point = "point_pair_{name}".format(name=name)
    constraint_name_ori = "ori_pair_{name}".format(name=name)
    if make_error:
        error_val = "\nerror_target = error_target + abs(dist_{constraint_name_point})+abs(orientation_{constraint_name_ori})".format(
            constraint_name_point=constraint_name_point, constraint_name_ori=constraint_name_ori)
        error_statement = error_val+'\nctx:setOutputExpression("error",error_target)'
    pair_constraint = make_point_pair_constraint(framer1.object, framer2.object, constraint_name_point, constraint_name_point, make_error=False, 
                                                 point1=point1, point2=point2, activation=activation)
    ori_constraint = make_orientation_constraint(framer1, framer2, name=constraint_name_ori, constraint_name=constraint_name_ori,
                                                 make_error=False, activation=activation)
    return pair_constraint + "\n" + ori_constraint + error_statement


def make_collision_constraints_listed(collision_list, K="K"):
    constraint_text = "\n"
    for ctuple in collision_list:
        constraint_text += make_distance_bound_constraint(ctuple[0], ctuple[1], K=K)
    return constraint_text


def make_collision_constraints(geometry_items1, geometry_items2=None, K="K", min_distance_map=None):
    constraint_text = "\n"
    idx1 = 0
    for ctem1 in geometry_items1:
        idx1 += 1
        if geometry_items2 is None:
            geometry_items_tmp = geometry_items1[idx1:]
        else:
            geometry_items_tmp = geometry_items2

        for ctem2 in geometry_items_tmp:
            if ctem2.link_name in ctem1.adjacent_links or ctem1.link_name in ctem2.adjacent_links:
                continue
            else:
                if min_distance_map is not None:
                    min_link_dist = min_distance_map[ctem1.link_name][ctem2.link_name]
                    min_col_dist = min_link_dist - (np.linalg.norm(ctem1.get_off_max()) + np.linalg.norm(ctem2.get_off_max()))
                    if min_col_dist > 0:
                        continue
                constraint_text += make_distance_bound_constraint(ctem1, ctem2, K=K)
    return constraint_text

def get_online_input_text(ctems):
    obs_tf_text = ""
    kwargs_online = {}
    online_names = []
    for gtem in ctems:
        if gtem.online:
            online_names.append(gtem.name)
            obs_tf_text += """
            oln_{name}_x = ctx:createInputChannelScalar("oln_{name}_x", 0)
            oln_{name}_y = ctx:createInputChannelScalar("oln_{name}_y", 0)
            oln_{name}_z = ctx:createInputChannelScalar("oln_{name}_z", 0)
            {tf_name} = translate_x(oln_{name}_x)*translate_y(oln_{name}_y)*translate_z(oln_{name}_z)
            """.format(name=gtem.name, tf_name=get_tf_name(gtem))
            kwargs_online.update(dict(inp_lbl=['oln_{name}_{axis}'.format(name=gtem.name,
                                                                       axis=axis) for axis in "xyz"],
                                   inp=gtem.get_offset_tf()[:3, 3].tolist()))
    return obs_tf_text, kwargs_online, online_names


def make_joint_constraints(joint_names, make_error=True, priority=2, K_joint="K", activation=False):
    joint_constraints = ""
    error_statement = ""
    if activation:
        joint_constraints += 'joint_activation = ctx:createInputChannelScalar("joint_activation",0.0) \n'
        activation_expr = 'joint_activation*'
    else:
        activation_expr = ''

    for i in range(len(joint_names)):
        joint_constraints += """
target_{joint_name} = ctx:createInputChannelScalar("target_{joint_name}",0.0)
error_{joint_name} = robot_jval[{index}]-target_{joint_name}
Constraint{{
    context=ctx,
    name="constraint_{joint_name}",
    expr={activation_expr}error_{joint_name},
    priority    = {priority},
    K           = {K_joint}
}}
            """.format(joint_name=joint_names[i], index=i+1, priority=priority, K_joint=K_joint,
                       activation_expr=activation_expr)
        if make_error:
            error_statement += 'abs(error_{joint_name})+'.format(joint_name=joint_names[i])
    if make_error:
        error_val = "\nerror_target = error_target + "+error_statement[:-1]
        error_statement = error_val
    return joint_constraints + error_statement

def make_action_constraints(object, point_name, effector, point=None, activation=False):
    ac_point = object.action_points_dict[point_name]
    if isinstance(ac_point, FramedPoint):
        make_constraint_fun = make_oriented_point_constraint
    elif isinstance(ac_point, DirectedPoint):
        make_constraint_fun = make_directed_point_constraint
    const_txt = make_constraint_fun(ac_point.handle, effector, ac_point.name_constraint, point2=point, activation=activation)
    return const_txt


