(define (stream pnp-tamp)
  (:stream stream-time
    :inputs (?k)
    :domain (Timer ?k)
    :outputs (?i)
    :certified (Time ?i)
  )
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )
  (:stream check-feasible
    :inputs (?o ?p ?g ?i)
    :domain (and (Pose ?o ?p) (Grasp ?o ?g) (Time ?i))
    :outputs (?e)
    :certified (and (EndPose ?e) (Feasible ?o ?p ?g ?e))
  )
  (:stream plan-free-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :fluents (AtPose) ; AtGrasp
    :outputs (?t)
    :certified (FreeMotion ?q1 ?t ?q2)
  )
  (:stream plan-approach-motion
    :inputs (?q1 ?e2)
    :domain (and (Conf ?q1) (EndPose ?e2))
    :fluents (AtPose) ; AtGrasp
    :outputs (?q2 ?t)
    :certified (and (Conf ?q2) (Kin ?e2 ?q2) (FreeMotion ?q1 ?t ?q2))
  )
  (:stream plan-holding-motion
    :inputs (?q1 ?e2 ?o ?g)
    :domain (and (Conf ?q1) (EndPose ?e2) (Grasp ?o ?g))
    :fluents (AtPose)
    :outputs (?q2 ?t)
    :certified (and (Conf ?q2) (Kin ?e2 ?q2) (HoldingMotion ?q1 ?t ?q2 ?o ?g))
  )

  (:stream test-cfree-pose-pose
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?o1 ?p1 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeApproachPose ?o1 ?p1 ?g1 ?o2 ?p2)
  )
  (:stream test-cfree-traj-pose
    :inputs (?t ?o2 ?p2)
    :domain (and (Traj ?t) (Pose ?o2 ?p2))
    :certified (CFreeTrajPose ?t ?o2 ?p2)
  )
)