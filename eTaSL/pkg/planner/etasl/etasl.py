from __future__ import print_function
from etasl_py.etasl import etasl_simulator, EventException

from ...joint_utils import get_joint_names_csv, joint_list2dict, get_min_distance_map
from .constraint_etasl import *
from ..interface import PlannerInterface
from copy import deepcopy

K_DEFAULT = 10

def augment_jnames_dot(joint_names):
    return np.concatenate([[jname, jname + "_dot"] for jname in joint_names], axis=0).tolist()

def augment_jvals_dot(jvals, jdots=None):
    if jdots is None:
        jdots = np.zeros_like(jvals)
    return np.concatenate([[jval, jdot] for jval, jdot in zip(jvals, jdots)], axis=0)

class etasl_planner(PlannerInterface):
    def __init__(self, joint_names, link_names, urdf_path,
                          nWSR=300, cputime=1000, regularization_factor= 1e-6, timescale=0.25):
        self.joint_names, self.link_names, self.urdf_path= joint_names, link_names, urdf_path
        self.nWSR, self.cputime, self.regularization_factor = nWSR, cputime, regularization_factor
        self.init_text = self.get_init_text(timescale=timescale)
        self.ghnd = GeometryHandle.instance()

    def update_gtems(self):
        self.ghnd.update()
        self.min_distance_map = get_min_distance_map()
        self.item_text = get_item_text(self.ghnd)
        self.fixed_tf_text = get_tf_text(self.ghnd.fixed_gtems)
        self.online_input_text, self.kwargs_online, self.online_names = \
            get_online_input_text(self.ghnd)
        self.fixed_collision_text = make_collision_constraints(self.ghnd.fixed_ctems,
                                                               min_distance_map=self.min_distance_map)

    def plan_transition(self, from_state, to_state, binding_list, vel_conv=1e-2, err_conv=1e-4, collision=True,
                        N=1, dt=1e-2, print_expression=False, cut_dot=False, **kwargs):
        full_context, kwargs = self.get_transition_context(
            from_state, to_state, binding_list, vel_conv, err_conv, collision=collision, **kwargs)
        if print_expression:
            print(full_context)
        e = self.set_simulate(full_context, initial_jpos=np.array(from_state.Q),
                         N=N, dt=dt, cut_dot=cut_dot, **kwargs)
        error = e.error if hasattr(e, 'error') else None
        success = error<err_conv if error is not None else False
        return e.POS, e.POS[-1], error, success

    def get_transition_context(self, from_state=None, to_state=None, binding_list=[], vel_conv=1e-2, err_conv=1e-4, collision=True,
                               **kwargs):
        kwargs.update(deepcopy(self.kwargs_online))

        tf_text = self.fixed_tf_text + self.online_input_text + get_tf_text(self.ghnd.movable_gtems)

        if collision:
            col_text = self.fixed_collision_text + \
                       make_collision_constraints(self.ghnd.fixed_ctems, self.ghnd.movable_ctems,
                                                                   min_distance_map=self.min_distance_map) + \
                       make_collision_constraints(self.ghnd.movable_ctems,
                                                                   min_distance_map=self.min_distance_map)
        else:
            col_text = ""

        additional_constraints = ""
        for bd1 in binding_list:
            additional_constraints += make_action_constraints(self.object_dict[bd1[0]], bd1[1], self.binder_dict[bd1[2]].effector,
                                                              point=self.binder_dict[bd1[2]].point)

        if additional_constraints=="" and to_state.Q is not None and np.sum(np.abs(np.subtract(to_state.Q,from_state.Q)))>1e-2:
            additional_constraints=make_joint_constraints(joint_names=self.joint_names)
            kwargs_new = dict(inp_lbl=['target_%s'%jname for jname in self.joint_names],
                                       inp= list(to_state.Q))

            for k, v in kwargs_new.items():
                if k in kwargs:
                    if isinstance(v, list) and isinstance(v, list):
                        kwargs[k] += v
                    elif isinstance(v, dict) and isinstance(v, dict):
                        kwargs[k].update(v)
                    else:
                        kwargs[k] = v
                else:
                    kwargs[k] = v

            self.gtimer.toc("make_joint_constraints")
        return self.get_full_context(self.init_text + self.item_text + tf_text+col_text,
                                additional_constraints, vel_conv, err_conv), kwargs

    def get_init_text(self, timescale=0.25, K_default=K_DEFAULT):
        jnames_format = get_joint_names_csv(self.joint_names)
        # define margin and translation
        transform_text = """
    margin=0.0001
    radius=0.0
    error_target=0
    """
        Texpression_text = ""
        for lname in self.link_names:
            transform_text += 'u:addTransform("{T_link_name}","{link_name}","world")\n'.format(T_link_name=get_link_transformation(lname), link_name=lname)
            Texpression_text += '{T_link_name} = r.{T_link_name}\n'.format(T_link_name=get_link_transformation(lname))


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
        """.format(timescale=str(timescale) if timescale is not None else "", urdf_path=self.urdf_path,
                   transform_text=transform_text, Texpression_text=Texpression_text, jnames_format=jnames_format,
                   K_default=K_default)

        return definition_text

    def get_simulation(self, init_text):
        self.etasl = etasl_simulator(nWSR=self.nWSR, cputime=self.cputime, regularization_factor= self.regularization_factor)
        self.etasl.readTaskSpecificationString(init_text)
        return self.etasl

    def simulate(self, initial_jpos, joint_names = None, initial_jpos_dot=None,
                 inp_lbl=[], inp=[], N=100, dt=0.02, cut_dot=False):
        if joint_names is None:
            joint_names = self.joint_names
        if initial_jpos_dot is None:
            initial_jpos_dot = np.zeros_like(initial_jpos)
        pos_lbl = augment_jnames_dot(joint_names)
        initial_jpos_exp = augment_jvals_dot(initial_jpos, initial_jpos_dot)
        self.etasl.setInputTable(inp_lbl,inp)
        try:
            try:
                self.etasl.initialize(initial_jpos_exp, pos_lbl)
                self.etasl.simulate(N=N,dt=dt)
            except EventException as e:
                idx_end = np.where(np.any(self.etasl.VEL!=0,axis=1))[0]
                if len(idx_end)>0 and hasattr(self.etasl, 'POS'):
                    idx_end = idx_end[-1]
                    if cut_dot:
                        self.etasl.VEL = integrate(self.etasl.VEL[:idx_end+1,1::2], dt)
                        self.etasl.POS = integrate(self.etasl.VEL, dt, initial_jpos)
                    else:
                        self.etasl.POS = self.etasl.POS[:idx_end+1, ::2]
                        self.etasl.VEL = self.etasl.VEL[:idx_end+1, ::2]
                    self.etasl.TIME = self.etasl.TIME[:idx_end+1]
                    self.etasl.OUTP = self.etasl.OUTP[:idx_end+1]
                return
            if cut_dot:
                self.etasl.VEL = integrate(self.etasl.VEL[:,1::2], dt)
                self.etasl.POS = integrate(self.etasl.VEL, dt, initial_jpos)
            else:
                self.etasl.POS = self.etasl.POS[:, ::2]
                self.etasl.VEL = self.etasl.VEL[:, ::2]
        except Exception as e:
            print('unknown eTaSL exception: {}'.format(str(e)))

    def do_simulate(self, **kwargs):
        self.simulate(**kwargs)
        if hasattr(self.etasl, 'POS') and self.etasl.POS is not None and len(self.etasl.POS)>0:
            self.etasl.joint_dict_last = joint_list2dict(self.etasl.POS[-1], self.joint_names)
        output = self.etasl.etasl.getOutput()
        if 'global.error' in output:
            self.etasl.error = output['global.error']
        return self.etasl

    def set_simulate(self, full_context, initial_jpos=[], **kwargs):
        self.etasl = self.get_simulation(full_context)
        return self.do_simulate(initial_jpos=initial_jpos, **kwargs)

    def get_full_context(self, init_text, additional_constraints="", vel_conv="1E-2", err_conv="1E-5"):

        vel_statement=""
        for i in range(len(self.joint_names)):
            vel_statement += 'abs(previous_velocity(time, robot_jval[{index}]))+'\
                .format(index = i+1, joint_name=self.joint_names[i])
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

def get_item_text(geo_list):
    item_text = "\n"
    for gtem in geo_list:
        item_text += """{name}={ctem}\n""".format(name=gtem.name, ctem=get_representation(gtem))
    return item_text

def get_tf_text(geo_list):
    transformation_text = ""
    for ctem in geo_list:
        if not ctem.online:
            transformation_text += get_tf_representation(ctem)+"\n"
    return transformation_text

