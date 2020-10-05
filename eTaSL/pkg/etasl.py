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
K_DEFAULT = 10

def augment_jnames_dot(joint_names):
    return np.concatenate([[jname, jname + "_dot"] for jname in joint_names], axis=0).tolist()
def augment_jvals_dot(jvals, jdots=None):
    if jdots is None:
        jdots = np.zeros_like(jvals)
    return np.concatenate([[jval, jdot] for jval, jdot in zip(jvals, jdots)], axis=0)

def set_simulation_config(joint_names, link_names, urdf_content, urdf_path, geometry_items_dict=None,
                          nWSR=300, cputime=1000, regularization_factor= 1e-6):
    global JOINT_NAMES_SIMULATION, LINK_NAMES_SIMULATION, URDF_CONTENT_SIMULATION, URDF_PATH_SIMULATION, COLLISION_ITEMS_DICT,\
            NWSR, CPUTIME, REG_FACTOR
    JOINT_NAMES_SIMULATION = joint_names
    LINK_NAMES_SIMULATION = link_names
    URDF_CONTENT_SIMULATION = urdf_content
    URDF_PATH_SIMULATION = urdf_path
    COLLISION_ITEMS_DICT = geometry_items_dict
    NWSR, CPUTIME, REG_FACTOR = nWSR, cputime, regularization_factor

def get_init_text(timescale=0.25, K_default=K_DEFAULT):
    global JOINT_NAMES_SIMULATION, LINK_NAMES_SIMULATION, URDF_CONTENT_SIMULATION, URDF_PATH_SIMULATION, COLLISION_ITEMS_DICT
    joint_names = JOINT_NAMES_SIMULATION
    link_names = LINK_NAMES_SIMULATION
    urdf_content = URDF_CONTENT_SIMULATION
    urdf_path = URDF_PATH_SIMULATION
    jnames_format = get_joint_names_csv(joint_names, urdf_content)
    # define margin and translation
    transform_text = """
margin=0.0001
radius=0.0
error_target=0
"""
    Texpression_text = ""
    for lname in link_names:
        transform_text += 'u:addTransform("{T_link_name}","{link_name}","world")\n'.format(T_link_name=get_transformation(lname), link_name=lname)
        Texpression_text += '{T_link_name} = r.{T_link_name}\n'.format(T_link_name=get_transformation(lname))


    definition_text = """
require("context")
require("geometric")
--require("libexpressiongraph_collision")
require("collision")
require("libexpressiongraph_velocities")
local u=UrdfExpr({timescale});
local fn = "{urdf_path}"
u:readFromFile(fn)
{transform_text}
local r = u:getExpressions(ctx)
{Texpression_text}
robot_jname={{{jnames_format}}}
robot_jval = {{}}
for i=1,#robot_jname do
   robot_jval[i]   = ctx:getScalarExpr(robot_jname[i])
end

K={K_default}
    """.format(timescale=str(timescale) if timescale is not None else "", urdf_path=urdf_path,
               transform_text=transform_text, Texpression_text=Texpression_text, jnames_format=jnames_format,
               K_default=K_default)
    
    return definition_text

def get_item_text(geo_list):
    item_text = "\n"
    for gtem in geo_list:
        item_text += """{name}={ctem}\n""".format(name=gtem.name, ctem=gtem.get_representation())
    return item_text

def get_tf_text(geo_list):
    transformation_text = ""
    for ctem in geo_list:
        transformation_text += ctem.get_tf_representation()+"\n"
    return transformation_text

def get_simulation(init_text):
    etasl = etasl_simulator(nWSR=NWSR, cputime=CPUTIME, regularization_factor= REG_FACTOR)
    etasl.readTaskSpecificationString(init_text)
    return etasl

def simulate(etasl, initial_jpos, joint_names = None, initial_jpos_dot=None,
             inp_lbl=[], inp=[], N=100, dt=0.02, cut_dot=True):
    if joint_names is None:
        joint_names = JOINT_NAMES_SIMULATION
    if initial_jpos_dot is None:
        initial_jpos_dot = np.zeros_like(initial_jpos)
    pos_lbl = augment_jnames_dot(joint_names)
    initial_jpos_exp = augment_jvals_dot(initial_jpos, initial_jpos_dot)
    etasl.setInputTable(inp_lbl,inp)

    try:
        try:
            etasl.initialize(initial_jpos_exp, pos_lbl)
            etasl.simulate(N=N,dt=dt)
        except EventException as e:
            idx_end = np.where(np.any(etasl.VEL!=0,axis=1))[0]
            if len(idx_end)>0 and hasattr(etasl, 'POS'):
                idx_end = idx_end[-1]
                if cut_dot:
                    etasl.VEL = integrate(etasl.VEL[:idx_end+1,1::2], dt)
                    etasl.POS = integrate(etasl.VEL, dt, initial_jpos)
                else:
                    etasl.POS = etasl.POS[:idx_end+1]
                    etasl.VEL = etasl.VEL[:idx_end+1]
                etasl.TIME = etasl.TIME[:idx_end+1]
                etasl.OUTP = etasl.OUTP[:idx_end+1]
            return
        if cut_dot:
            etasl.VEL = integrate(etasl.VEL[:,1::2], dt)
            etasl.POS = integrate(etasl.VEL, dt, initial_jpos)
    except Exception as e:
        print('unknown eTaSL exception: {}'.format(str(e)))
        
def get_full_context(init_text, additional_constraints="", vel_conv="1E-2", err_conv="1E-5"):

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
        """.format(vel_conv=0)
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
        monitor_string += \
            """
            vel_error = maximum(1-time, abs(previous_velocity(time, error_target)))
            ctx:setOutputExpression("vel_error",vel_error)
            Monitor {{
                context = ctx,
                name = "converged",
                expr   = vel_error/error_target,
                lower = {vel_conv},
                actionname ="exit",
                argument = "converged"
            }}
            """.format(vel_conv=vel_conv)
    return init_text + "\n" + additional_constraints + "\n" + monitor_string
    
def do_simulate(etasl, **kwargs):
    simulate(etasl=etasl, **kwargs)
    if hasattr(etasl, 'POS') and etasl.POS is not None and len(etasl.POS)>0:
        etasl.joint_dict_last = joint_list2dict(etasl.POS[-1], JOINT_NAMES_SIMULATION)
    output = etasl.etasl.getOutput()
    if 'global.error' in output:
        etasl.error = output['global.error']
    return etasl

def set_simulate(full_context, initial_jpos=[], **kwargs):
    etasl = get_simulation(full_context)
    return do_simulate(etasl, initial_jpos=initial_jpos, **kwargs)