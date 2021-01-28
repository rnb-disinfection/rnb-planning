from __future__ import print_function
from etasl_py.etasl import etasl_simulator, EventException

from ....utils.joint_utils import get_joint_names_csv
from ....utils.utils import integrate, list2dict
from .constraint_etasl import *
from ..interface import MotionInterface, downample_traj
from copy import deepcopy
from collections import defaultdict

K_DEFAULT = 30
TRAJ_RADII_MAX = np.deg2rad(10)
DEFAULT_TRAJ_COUNT = 50

def augment_jnames_dot(joint_names):
    return np.concatenate([[jname, jname + "_dot"] for jname in joint_names], axis=0).tolist()

def augment_jvals_dot(jvals, jdots=None):
    if jdots is None:
        jdots = np.zeros_like(jvals)
    return np.concatenate([[jval, jdot] for jval, jdot in zip(jvals, jdots)], axis=0)

##
# @class EtaslPlanner
# @brief eTaSL motion planner
class EtaslPlanner(MotionInterface):
    NAME = 'eTaSL'

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene, nWSR=300, cputime=200, regularization_factor= 1e-6, timescale=0.25):
        MotionInterface.__init__(self, pscene)
        self.nWSR, self.cputime, self.regularization_factor = nWSR, cputime, regularization_factor
        self.init_text = self.__get_init_text(timescale=timescale)

    ##
    # @brief update changes in geometric scene and prepare collision context
    def update_gscene(self):
        self.gscene.update()
        self.min_distance_map = self.gscene.min_distance_map
        self.item_text = get_item_text(self.gscene)
        self.fixed_tf_text = get_tf_text(self.gscene.fixed_gtems)
        self.online_input_text, self.kwargs_online, self.online_names = \
            get_online_input_text(self.gscene)
        self.fixed_collision_text = make_collision_constraints(self.gscene.fixed_ctems,
                                                               min_distance_map=self.min_distance_map)

    ##
    # @brief eTaSL planning implementation
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param binding_list   list of bindings to pursue
    # @param redundancy_dict    not supported
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_dict=None,
                       vel_conv=1e-2, err_conv=1e-3, collision=True, N=1000, dt=1e-2,
                       print_expression=False, cut_dot=False, traj_count=DEFAULT_TRAJ_COUNT,
                       timeout=None, **kwargs):
        if redundancy_dict is not None:
            raise(NotImplementedError("Fixed redundancy is not implemented in eTaSL"))
        if len(binding_list)>1:
            print("===================== plan simultaneous manipulation =====================")
        if len(binding_list)==0:
            print("===================== plan joint manipulation =====================")
        full_context, kwargs = self.__get_transition_context(
            from_state, to_state, binding_list, vel_conv, err_conv, collision=collision, **kwargs)
        if print_expression:
            print(full_context)
        e = self.__set_simulate(full_context, initial_jpos=np.array(from_state.Q),
                         N=N, dt=dt, cut_dot=cut_dot, **kwargs)
        error = e.error if hasattr(e, 'error') else None
        POS = e.POS if hasattr(e, 'POS') else []
        POS_last = e.POS[-1] if hasattr(e, 'POS') else [0]*self.joint_num
        success = error<err_conv if error is not None else False
        if len(POS)>0 and traj_count:
            POS = downample_traj(POS, traj_count)
        return POS, POS_last, error, success

    ##
    # @brief initialize online eTaSL planning
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    def init_online_algorithm(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        dt = 1.0 / control_freq
        dt_sim = dt * playback_rate
        N = int(float(T_step) / dt_sim)
        pos_start = from_state.Q
        self.err_conv = kwargs['err_conv']

        full_context, kwargs = \
            self.__get_transition_context(from_state, to_state, binding_list,
                                        N=N, dt=dt_sim, activation=(from_state.binding_state != to_state.binding_state),
                                        **kwargs)

        if "inp_lbl" not in kwargs:
            kwargs["inp_lbl"] = []
        if "inp" not in kwargs:
            kwargs["inp"] = []
        self.jact_idx = -1
        self.cact_idx = -1
        if from_state.binding_state != to_state.binding_state:
            joint_context = make_joint_constraints(self.joint_names, priority=2, make_error=False, activation=True)
            kwargs["inp_lbl"] = list(kwargs["inp_lbl"]) + ["target_{joint_name}".format(joint_name=joint_name) for joint_name in self.joint_names]
            kwargs["inp_lbl"] += ['joint_activation', 'constraint_activation']
            kwargs["inp"] = list(kwargs["inp"]) + list(from_state.Q)
            kwargs["inp"] += [0, 0]
            self.jact_idx = kwargs["inp_lbl"].index('joint_activation')
            self.cact_idx = kwargs["inp_lbl"].index('constraint_activation')
        else:
            joint_context = ""

        full_context += joint_context
        self.e_sim = self.__get_simulation(full_context)


        self.inp_lbl = kwargs['inp_lbl'] if 'inp_lbl' in kwargs else []
        self.inp = np.array(kwargs['inp'] if 'inp' in kwargs else [])
        # print("self.inp_lbl: {}".format(self.inp_lbl))
        # print("self.inp: {}".format(self.inp))

        self.e_sim.setInputTable(self.inp_lbl, inp=self.inp)

        self.pos_lbl = augment_jnames_dot(self.joint_names)
        initial_jpos_exp = augment_jvals_dot(pos_start, np.zeros_like(pos_start))
        self.e_sim.initialize(initial_jpos_exp, self.pos_lbl)

        pos = self.e_sim.simulate_begin(N, dt_sim)
        self.e_sim.DT = dt_sim
        self.kwargs_online_tmp = kwargs
        self.idx_jnt_online = [self.inp_lbl.index("target_{joint_name}".format(joint_name=joint_name)) for joint_name in self.joint_names]
        self.VEL_CUR = np.zeros_like(pos_start)
        self.POS_CUR = pos_start

        return pos, binding_list

    def step_online_plan(self, i_q, pos, wp_action=False):
        end_loop = False
        try:
            if self.jact_idx>=0 and self.cact_idx>=0:
                self.inp[self.jact_idx] = int(wp_action)
                self.inp[self.cact_idx] = int(not wp_action)
                # print("{}: {}".format(wp_action, self.inp[[self.jact_idx, self.cact_idx]]))
            pos = self.e_sim.simulate_step(i_q, pos, dt=None, inp_cur=self.inp)
            # self.VEL_CUR = self.VEL_CUR + self.e_sim.VEL[i_q, 1::2] * self.e_sim.DT
            # self.POS_CUR = self.POS_CUR + self.VEL_CUR * self.e_sim.DT
            self.VEL_CUR = self.e_sim.POS[i_q, 1::2]
            self.POS_CUR = self.e_sim.POS[i_q, 0::2]
            # pos = list2dict(augment_jvals_dot(self.POS_CUR, self.VEL_CUR),
            #                       self.pos_lbl)
        except EventException as e:
            err_msg = str(e)
            if len(err_msg)>100:
                err_msg = err_msg[:100]
            print('step eTaSL exception: {}'.format(err_msg))
            end_loop = True
        eout = self.e_sim.etasl.getOutput()
        error = eout['global.error'] if 'global.error' in eout else None
        success =  error < self.err_conv*2 if error is not None else False
        return pos, end_loop, error, success


    def update_online(self, obsPos_dict):
        for k, v in obsPos_dict.items():
            _pos = v[:3, 3]
            for _p, _k in zip(_pos, ["oln_{name}_{axis}".format(name=k, axis=axis) for axis in "xyz"]):
                if _k in self.inp_lbl:
                    self.inp[self.inp_lbl.index(_k)] = _p

    def update_target_joint(self, idx_cur, traj, joint_cur):
        error_max = np.max(np.abs(joint_cur-traj[idx_cur]))
        # print("joints: {}".format(joint_cur))
        # print("traj: {}".format(traj[idx_cur]))
        # print("error: {}".format(error))
        if error_max < TRAJ_RADII_MAX:
            if idx_cur+1 < len(traj):
                idx_cur += 1
                # print("traj: {}".format(traj[idx_cur]))
        self.inp[self.idx_jnt_online] = traj[idx_cur]
        # self.inp[self.idx_jnt_online] = traj[-1]
        return idx_cur # len(traj)

    def __get_transition_context(self, from_state=None, to_state=None, binding_list=[],
                               vel_conv=1e-2, err_conv=1e-4, collision=True,
                               activation=False, redundancy_dict=None, **kwargs):
        kwargs.update(deepcopy(self.kwargs_online))

        tf_text = self.fixed_tf_text + self.online_input_text + get_tf_text(self.gscene.movable_gtems)

        if collision:
            col_text = self.fixed_collision_text + \
                       make_collision_constraints(self.gscene.fixed_ctems, self.gscene.movable_ctems,
                                                                   min_distance_map=self.min_distance_map) + \
                       make_collision_constraints(self.gscene.movable_ctems,
                                                                   min_distance_map=self.min_distance_map)
        else:
            col_text = ""

        additional_constraints = '\nconstraint_activation = ctx:createInputChannelScalar("constraint_activation",0.0) \n' if activation else ""
        for bd1 in binding_list:
            additional_constraints += make_action_constraints(
                self.pscene.subject_dict[bd1[0]].action_points_dict[bd1[1]], self.pscene.actor_dict[bd1[2]],
                redundancy=redundancy_dict[bd1[0]] if redundancy_dict else None, activation=activation)

        if additional_constraints=="" and to_state.Q is not None:# and np.sum(np.abs(np.subtract(to_state.Q,from_state.Q)))>1e-2:
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
        return self.__get_full_context(self.init_text + self.item_text + tf_text+col_text,
                                additional_constraints, vel_conv, err_conv), kwargs

    def __get_init_text(self, timescale=0.25, K_default=K_DEFAULT):
        jnames_format = get_joint_names_csv(self.joint_names)
        # define margin and translation
        transform_text = """
    margin=0.0001
    radius=0.0
    error_target=0
    """
        Texpression_text = ""
        for lname in self.link_names:
            transform_text += 'u:addTransform("{T_link_name}","{link_name}","base_link")\n'.format(T_link_name=get_link_transformation(lname), link_name=lname)
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

    def __get_simulation(self, init_text):
        self.etasl = etasl_simulator(nWSR=self.nWSR, cputime=self.cputime, regularization_factor= self.regularization_factor)
        self.etasl.readTaskSpecificationString(init_text)
        return self.etasl

    def __simulate(self, initial_jpos, joint_names = None, initial_jpos_dot=None,
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
                res = True
                return res
            if cut_dot:
                self.etasl.VEL = integrate(self.etasl.VEL[:,1::2], dt)
                self.etasl.POS = integrate(self.etasl.VEL, dt, initial_jpos)
            else:
                self.etasl.POS = self.etasl.POS[:, ::2]
                self.etasl.VEL = self.etasl.VEL[:, ::2]
            res = True
        except Exception as e:
            res = False
            err_msg = str(e)
            if len(err_msg)>100:
                err_msg = err_msg[:100]
            print('eTaSL exception: {}'.format(err_msg))
        return res

    def __do_simulate(self, **kwargs):
        res = self.__simulate(**kwargs)
        if res:
            if hasattr(self.etasl, 'POS') and self.etasl.POS is not None and len(self.etasl.POS)>0:
                self.etasl.joint_dict_last = list2dict(self.etasl.POS[-1], self.joint_names)
            output = self.etasl.etasl.getOutput()
            if 'global.error' in output:
                self.etasl.error = output['global.error']
        else:
            self.etasl.error = None
        return self.etasl

    def __set_simulate(self, full_context, initial_jpos, **kwargs):
        self.etasl = self.__get_simulation(full_context)
        return self.__do_simulate(initial_jpos=initial_jpos, **kwargs)

    def __get_full_context(self, init_text, additional_constraints="", vel_conv="1E-2", err_conv="1E-5"):

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
            ctx:setOutputExpression("error",error_target)
            ctx:setOutputExpression("vel_joints",vel_joints)
            Monitor {{
                context = ctx,
                name = "converged",
                expr   = vel_joints,
                lower = {vel_conv},
                actionname ="exit",
                argument = "converged"
            }}
            Monitor {{
                context = ctx,
                name = "goal_reached",
                expr   = error_target,
                lower = {err_conv},
                actionname ="exit",
                argument = "e_arrived"
            }}
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
            """.format(err_conv=err_conv, vel_conv=vel_conv)
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

