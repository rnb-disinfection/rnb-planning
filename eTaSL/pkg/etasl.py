from __future__ import print_function
import numpy as np
from etasl_py.etasl import etasl_simulator
from etasl_py.etasl import EventException
import time as timer

from .joint_utils import get_joint_names_csv, get_transformation, joint_list2dict
from .constraint_base import *

JOINT_NAMES_SIMULATION = None
LINK_NAMES_SIMULATION = None
URDF_CONTENT_SIMULATION = None
URDF_PATH_SIMULATION = None
COLLISION_ITEMS_DICT = None
NWSR = 100
CPUTIME =1000
REG_FACTOR =1e-6

def set_simulation_config(joint_names, link_names, urdf_content, urdf_path, collision_items_dict=None, 
                          nWSR=300, cputime=1000, regularization_factor= 1e-6):
    global JOINT_NAMES_SIMULATION, LINK_NAMES_SIMULATION, URDF_CONTENT_SIMULATION, URDF_PATH_SIMULATION, COLLISION_ITEMS_DICT,\
            NWSR, CPUTIME, REG_FACTOR
    JOINT_NAMES_SIMULATION = joint_names
    LINK_NAMES_SIMULATION = link_names
    URDF_CONTENT_SIMULATION = urdf_content
    URDF_PATH_SIMULATION = urdf_path
    COLLISION_ITEMS_DICT = collision_items_dict
    NWSR, CPUTIME, REG_FACTOR = nWSR, cputime, regularization_factor

def get_init_text(collision_items_dict=None, joint_names=None, link_names=None, urdf_content=None, urdf_path=None):
    global JOINT_NAMES_SIMULATION, LINK_NAMES_SIMULATION, URDF_CONTENT_SIMULATION, URDF_PATH_SIMULATION, COLLISION_ITEMS_DICT
    if collision_items_dict is None: collision_items_dict = COLLISION_ITEMS_DICT
    else: COLLISION_ITEMS_DICT = collision_items_dict
    if joint_names is None: joint_names = JOINT_NAMES_SIMULATION
    else: JOINT_NAMES_SIMULATION = joint_names
    if link_names is None: link_names = LINK_NAMES_SIMULATION
    else: LINK_NAMES_SIMULATION = link_names
    if urdf_content is None: urdf_content = URDF_CONTENT_SIMULATION
    else: URDF_CONTENT_SIMULATION = urdf_content
    if urdf_path is None: urdf_path = URDF_PATH_SIMULATION
    else: URDF_PATH_SIMULATION = urdf_path
    jnames_format = get_joint_names_csv(joint_names, urdf_content)
    # define margin and translation
    transform_text = \
    """
        margin=0.0001
        radius=0.0
        error_target=0
    """
    Texpression_text = ""
    item_text = "\n"
    ctem_list = []
    for c_key in link_names:
        ctems = collision_items_dict[c_key]
        transform_text += '    u:addTransform("{T_link_name}","{link_name}","world")\n'.format(T_link_name=get_transformation(c_key), link_name=c_key)
        Texpression_text += '    {T_link_name} = r.{T_link_name}\n'.format(T_link_name=get_transformation(c_key))
        for i in range(len(ctems)):
            ctem = ctems[i]
            item_text += """    {name}={ctem}\n""".format(name=ctem.name, ctem=ctem.get_representation())
            if ctem.collision:
                ctem_list += [ctem]


    definition_text = """
        require("context")
        require("geometric")
        --require("libexpressiongraph_collision")
        require("collision")
        require("libexpressiongraph_velocities")
        local u=UrdfExpr();
        local fn = "%s"
        u:readFromFile(fn)
    %s
        local r = u:getExpressions(ctx)
    %s
        robot_jname={%s}
        robot_jval = {}
        for i=1,#robot_jname do
           robot_jval[i]   = ctx:getScalarExpr(robot_jname[i])
        end

        K=40
    """%(urdf_path, transform_text, Texpression_text, jnames_format)
    
    constraint_text = make_collision_constraints(ctem_list)
    
    return definition_text + item_text + constraint_text

def get_simulation(init_text):
    etasl = etasl_simulator(nWSR=NWSR, cputime=CPUTIME, regularization_factor= REG_FACTOR)
    etasl.readTaskSpecificationString(init_text)
    return etasl

def simulate(etasl, initial_jpos, joint_names = None, 
             inp_lbl=[], inp=[], N=100, dt=0.02):
    if joint_names is None:
        joint_names = JOINT_NAMES_SIMULATION
    time = np.arange(0,N)*dt
    pos_lbl = joint_names
    # inp_lbl=['tgt_x','tgt_y','tgt_z']
    # inp=[0.5, 0.0, 0.0]
#     print("N: ", N)
#     print("dt: ", dt)
    
    etasl.setInputTable(inp_lbl,inp)
    etasl.initialize(np.array(initial_jpos), pos_lbl)
    
    try:
        etasl.simulate(N=N,dt=dt)
    except EventException as e:
#         print(e)
        idx_end = np.where(np.any(etasl.VEL!=0,axis=1))[0]
        if len(idx_end)>0:
            idx_end = idx_end[-1]
            etasl.POS = etasl.POS[:idx_end+1]
            etasl.VEL = etasl.VEL[:idx_end+1]
            etasl.TIME = etasl.TIME[:idx_end+1]
            etasl.OUTP = etasl.OUTP[:idx_end+1]
    
def set_simulate(init_text, initial_jpos=[], additional_constraints="", 
                 vel_conv="1E-2", err_conv="1E-5", **kwargs):
    etasl = get_simulation(init_text)
#     print("init_text")
#     print(init_text)
    etasl.readTaskSpecificationString(additional_constraints)
    
    vel_statement=""
    for i in range(len(JOINT_NAMES_SIMULATION)):
        vel_statement += 'abs(previous_velocity(time, robot_jval[{index}]))+'\
            .format(index = i+1, joint_name=JOINT_NAMES_SIMULATION[i])
    vel_val = """
        vel_joints = maximum(1-time, {})
        """.format(vel_statement[:-1])
#     vel_val = "\nvel_joints = normalized_velocity(time, robot_jval)"
    monitor_string = vel_val + \
        """
        ctx:setOutputExpression("vel_joints",vel_joints)
        Monitor {{
            context = ctx,
            name = "converged",
            expr   = vel_joints,
            lower = {vel_conv},
            actionname ="exit",
            argument = "converged"
        }}
        """.format(vel_conv=vel_conv)
    if "error_target" in additional_constraints:
        monitor_string += \
            """
            Monitor {{
                context = ctx,
                name = "goal_reached",
                expr   = error_target,
                lower = {err_conv},
                actionname ="exit",
                argument = "e_arrived"
            }}
            """.format(err_conv=err_conv)
#         print('additional_constraints')
#         print(additional_constraints)
#         print('monitor_string')
#         print(monitor_string)
    etasl.readTaskSpecificationString(monitor_string)
    simulate(etasl=etasl, initial_jpos=initial_jpos, **kwargs)
    etasl.joint_dict_last = joint_list2dict(etasl.POS[-1], JOINT_NAMES_SIMULATION)
    output = etasl.etasl.getOutput()
    if 'global.error' in output:
        etasl.error = output['global.error']
    return etasl