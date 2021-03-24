require "rttlib"
require "rttros" 
--rtt.setLogLevel("Warning")
rtt.setLogLevel("Warning")

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end
depl:import("rtt_ros")
depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("etasl_rtt")

etasl_rtt_dir = rttros.find_rospack_roslua("etasl_skills")
collision_dir = rttros.find_rospack_roslua("expressiongraph_collision")

function s( stringtable )
    local v = rtt.Variable("strings")
    v:fromtab(stringtable)
    return v
end

function d( floattable )
    local v = rtt.Variable("array")
    v:fromtab(floattable)
    return v
end


-- deploy solver:
    ros:import("etasl_solver_qpoases")
    depl:loadComponent("solver","etasl_solver_qpoases")
    solver = depl:getPeer("solver")
    solver:getProperty("max_iterations"):set(1000)
    solver:getProperty("regularization"):set(0.003)
 
-- jointstate I/O factory    
    ros:import("etasl_iohandler_jointstate")
    depl:loadComponent("jointstate","Etasl_IOHandler_Jointstate")
    jointstate = depl:getPeer("jointstate")



-- deploy etasl component:
    depl:import("etasl_rtt")
    depl:loadComponent("moving_down", "etasl_rtt")
    -- create LuaComponents
    moving_down = depl:getPeer("moving_down")
    depl:connectPeers("moving_down","solver")
    solver:create_and_set_solver("moving_down")


    -- deploy simulated robot:
    depl:loadComponent("simrobot", "OCL::LuaComponent")
    simrobot = depl:getPeer("simrobot")
    simrobot:exec_file(etasl_rtt_dir.."/scripts/rtt/simple_robot_sim.lua")
    init_jnts=d{0.0, 0.0, 0.0, 1.9402476228540957, 0.0, -0.9261415142768581, 0.0}
    simrobot:getProperty("initial_position"):set( init_jnts )


    -- configure for an UR10 robot:
    moving_down:readTaskSpecificationFile(collision_dir.."/scripts/etasl/robot_flexfellow.lua")
    moving_down:readTaskSpecificationFile(collision_dir.."/scripts/etasl/moving.lua")
    jn = s{ "lbr_iiwa_joint_1", "lbr_iiwa_joint_2", "lbr_iiwa_joint_3", 
             "lbr_iiwa_joint_4", "lbr_iiwa_joint_5", "lbr_iiwa_joint_6", 
             "lbr_iiwa_joint_7"
         }
    moving_down:add_controller_inputport("jointpos","Joint position values",jn)
    moving_down:add_controller_outputport("jointvel","Joint velocity values",jn)
    depl:connectPeers("jointstate","moving_down")
    jointstate:add_controller_jointstate_output("moving_down","joint_state","Joint state value for the controller",jn)
 
    moving_down:add_etaslvar_outputport("etaslvar","",s{"global.d13","global.d14","global.d15"})
-- deploy reporter:
    depl:loadComponent("Reporter","OCL::FileReporting")
    reporter=depl:getPeer("Reporter")
    depl:connectPeers("moving_down","Reporter")
    reporter:reportPort("moving_down","jointvel")
    reporter:reportPort("moving_down","etaslvar")
    depl:connectPeers("simrobot","Reporter")
    reporter:reportPort("simrobot","jointpos")
    reporter:getProperty("ReportFile"):set("my_report.dat")


-- set activities:
    depl:setActivity("moving_down", 0.01, 50, rtt.globals.ORO_SCHED_RT)
    depl:setActivity("simrobot", 0.01, 50, rtt.globals.ORO_SCHED_RT)


-- deploy supervisor:
    depl:loadComponent("Supervisor", "OCL::LuaComponent")
    sup = depl:getPeer("Supervisor")
     
    sup:exec_file(etasl_rtt_dir.."/scripts/rtt/fsm_component.lua")
    sup:getProperty("state_machine"):set(collision_dir.."/scripts/rfsm/fsm.lua")
    sup:addPeer(depl)
    sup:configure()
    sup:start()
    cmd = rttlib.port_clone_conn(sup:getPort("events"))

-- connect ports:
    cp=rtt.Variable("ConnPolicy")
    depl:connect("moving_down.jointvel","simrobot.jointvel",cp ) 
    depl:connect("simrobot.jointpos","moving_down.jointpos",cp )
    depl:connect("moving_down.eventPort","Supervisor.events",cp)

    depl:stream("moving_down.joint_state", ros:topic("/joint_states"))


