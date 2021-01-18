-- example for defining a trapezoidal motion profile.
--
-- E. Aertbelien (c) 2016
--
require("context")

-- create class to encapsulate trapezoidal motionprofile
mp = create_motionprofile_trapezoidal()
-- set the progress variable: in this case: time
-- (but this can also be something else!)
mp:setProgress(time)

maxvel = constant(1.5)
maxacc = constant(0.5)

-- add a degree of freedom that starts at -1, ends at 2.0 with the given
-- maximum velocity and acceleration:
mp:addOutput( constant(-1.0), constant(2.0), maxvel, maxacc )
-- add another degree of freedom:
mp:addOutput( constant(-2.0), constant(2.0), maxvel, maxacc )

-- gets an expression for the first degree of freedom:
traj1 = get_output_profile(mp,0)
-- gets an expression for the second degree of freedom:
traj2 = get_output_profile(mp,1)



