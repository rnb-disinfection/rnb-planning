require("rtt")
require("rttlib")

tc          = rtt.getTC() 
depl        = tc:getPeer("Deployer")
moving_down = depl:getPeer("moving_down")
simrobot    = depl:getPeer("simrobot")
reporter    = depl:getPeer("Reporter")

pos1   = { 0/180*math.pi, -110/180*math.pi, 110/180*math.pi, -90/180*math.pi, -90/180*math.pi, 0/180*math.pi }  
pos2   = { 0/180*math.pi, -120/180*math.pi, 135/180*math.pi, -115/180*math.pi, -90/180*math.pi, 0/180*math.pi }  


return rfsm.state {
   configured = rfsm.state {
      entry=function() 
            simrobot:configure()
            reporter:configure() 
            simrobot:start()
            reporter:start() 
            
            moving_down:configure()
          end,
   },
 
   moving_A = rfsm.state {
      entry=function()  
            moving_down:activation_command("+global.moving_left")
            moving_down:readTaskSpecificationString("print(ctx)")
            moving_down:initialize()
            moving_down:start()
      end,
   },
   finished_A = rfsm.state {
        entry=function()  
            moving_down:activation_command("-global.moving_left")
        end,
   },
 
   moving_B = rfsm.state {
      entry=function()  
            moving_down:activation_command("+global.moving_back")
            moving_down:readTaskSpecificationString("print(ctx)")
            moving_down:initialize()
            moving_down:start()
      end,
   },
   finished_B = rfsm.state {
        entry=function()  
            moving_down:activation_command("-global.moving_back")
        end,
   },


   finished = rfsm.state {
        entry = function()
        end,
   },  

   rfsm.trans {src="initial", tgt="configured" },
   rfsm.trans {src="configured", tgt="moving_A", events={}},

   rfsm.trans {src="moving_A", tgt="finished_A", events={"e_finished@moving_down"}},
   rfsm.trans {src="finished_A", tgt="moving_B", events={}},
   rfsm.trans {src="moving_B", tgt="finished_B", events={"e_finished@moving_down"}},
   rfsm.trans {src="finished_B", tgt="finished", events={}},
 
}

