(define (stream pnp-tamp)
  (:stream sample-binding
    :inputs (?r ?a ?s)
    :domain (and (Robot ?r) (Actor ?r ?a) (Subject ?s))
    :outputs (?b)
    :certified (Binding ?s ?a ?b)
  )
  (:stream sample-transform
    :inputs (?r ?q ?s ?a ?b)
    :domain (and (Robot ?r) (Conf ?r ?q) (Actor ?r ?a) (Binding ?s ?a ?b))
    :outputs (?p)
    :certified (and (BindingPose ?s ?a ?b ?p) (Pose ?s ?p))
  )
  (:stream inverse-kinematics
    :inputs (?s ?p ?r ?a ?b)
    :domain (and (Binding ?s ?a ?b) (Pose ?s ?p) (Actor ?r ?a))
    :outputs (?q)
    :certified (and (Conf ?r ?q) (Kin ?s ?p ?a ?b ?r ?q))
  )
  (:stream plan-motion
    :inputs (?r ?q1 ?q2)
    :domain (and (Conf ?r ?q1) (Conf ?r ?q2))
    :outputs (?t)
    :certified (Motion ?r ?t ?q1 ?q2)
  )
  (:stream test-cfree-binding-binding
    :inputs (?s1 ?p1 ?s2 ?p2)
    :domain (and (Pose ?s1 ?p1) (Pose ?s2 ?p2))
    :certified (CFreePosePose ?s1 ?p1 ?s2 ?p2)
  )
  (:stream test-cfree-conf-pose
    :inputs (?r ?q ?s2 ?p2)
    :domain (and (Conf ?r ?q) (Pose ?s2 ?p2))
    :certified (CFreeConfPose ?r ?q ?s2 ?p2)
  )
  (:stream test-equal-pose-value
    :inputs (?s1 ?p1 ?s2 ?p2)
    :domain (and (Pose ?s1 ?p1) (Pose ?s2 ?p2))
    :certified (EqValPosePose ?p1 ?p2)
  )
)