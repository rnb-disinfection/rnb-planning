etasl_velocities package for dealing with velocities and velocity magnitudes
=============================================================================

A library for velocity-related constraints in eTaSL.  It consists of:

- special purpose expressiongraph nodes.  (This is independent of eTaSL and only dependent of expressiongraph/expressiongraph_lua).

- a lua library to facilitate velocity-related constraints.  (Dependencies on eTaSL due to the definition of the constraints).

The need of an expressiongraph node for velocity related constraints
---------------------------------------------------------------------

For scalar expressions, there is no need for a special purpose expressiongraph node. As long as one can write
an expression whose derivative is the entity we want to impose the constraint on, we can write the a constraint 
as follows:


.. code-block:: lua

        Constraint{
            context = ctx,
            name    = name,
            expr    = expr - constant(target_velocity)*time,
            target_lower = -1,
            target_upper = 1,
            K       = 0.0,
            weight  = weight,
            priority= 1 
        }

The expression for the constraint is such that its time derivative (the feed-forward for the
controller) corresponds to (d/dt expr - target_velocity).   Since the control constant K is zero, the
error itself is not taken into account. ``Scalar_Velocity_Limits`` encapsulates this trick into an easy to 
use function.


However, this method cannot be used for limits on the velocity magnitude of a 3D-vector, because the velocity *magnitude* is not
a derivative of some entity. the same problem exists for 2D-vectors and rotational velocities.

One could split up a vector into its components, but then the infinity norm of the velocity is limited, not the magnitude (i.e. 2-norm).

A solution to this problem is to project the velocity on to the current velocity direction.  The velocity is exactly what is
computed in the solver, and in the formulation of our control problem has to be linear in the  *velocities*.  This means that
we cannot directly or indirectly use the current velocity, and it is necessary to approximate this with a velocity estimated
from the previous time step.




Expressiongraph node
---------------------

An expressiongraph node that computes the last known velocity (i.e. time derivative) of an expression.
This is realized using an expressiongraph node with memory and additionally using time as an input:

- The first input of the node is the time. (in eTaSL the variable with name "time").

- The second input of the node can have different types: most important are Expression<Vector> and
  Expression<Rotation>

- Method used to compute the last known velocity:

  - Numerical differentiation,  the difference between the last known value and the current value of the expression, divided
    by the time difference.

  - Automatic differentiation would require some cooperation with the solver / solver-state, since the robot- and feature- variable velocities
    are required to compute the velocity by automatic differentiation.  Since it is the velocity of the previous time step that
    is computed, it is expected not to be a better estimate of the velocity than using numerical differentiation.

- Therefore, a *numerical differentiation* method is chosen.

- The expressiongraph node will return an *constant* expression with the (numerical) derivative as value.

- Unresolved question: Would it be usefull to also compute the (automatic) derivative for this computed numerical derivative?  Do we then always get what we expect to have? 

- The first time that the value of this node is requested, nothing is known about the previous value of the expression, and a *zero velocity* is 
  assumed.

- This node is created with the following lua statement: ``previous_velocity(time, expr)``

  eTaSL utility functions
------------------------

A few lua functions are defined to facilitate the use of the expression graph ndoe described above.

- There is also a utility function ``previous_velocity_direction(time,expr, eps)`` which returns a zero-vector when the velocity magnitude is below eps,
  and otherwise returns a normalized velocity.  If ``eps`` is not specified a default value of 1E-4 is used.

- An equivalent ``previous_rotational_velocity_direction(time,expr,eps)`` routine exists. 


- Velocity limits can be imposed by the following function:

.. code-block:: lua

        Velocity_Magnitude_Limits {
            context = ctx,
            name    = name,
            expr    = scalar_vector_or_rotation, 
            limit   = 1,                        -- in the appropriate units for expr.
            weight  = weight,
            priority= 1 
        }

- If expr in the statement above is a scalar function, the solution described above is used, it is a vector or rotation,
  a projection upon the last known (rotational) velocity is used.


Other uses of this library
---------------------------

- This library can also be used in surface tracking applications, e.g. to adapt your orientation
to the current velocity along a contour.

Caveat
-------

This library expects the following behaviour from the solver:

- all time steps are evaluated in order of increasing time.  


This could be a problem with solvers that use e.g. adaptive integration steps.

The current solvers have a fixed time-step and are evaluated in order of increasing time. 

License
-------

GNU LGPL v3, see LICENSE

Author
------

 E Aertbeliën, 2016.

