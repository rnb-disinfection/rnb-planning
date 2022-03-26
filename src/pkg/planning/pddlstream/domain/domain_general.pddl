(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Robot ?r)
    (Actor ?r ?a)
    (Subject ?s)
    (Conf ?r ?q)
    (Binding ?s ?a ?b)
    (Pose ?s ?p)
    (BindingPose ?s ?a ?b ?p)
    (Kin ?s ?p ?a ?b ?r ?q)
    (Motion ?r ?t ?q1 ?q2)

    (CFreePosePose ?s1 ?p1 ?s2 ?p2)
    (CFreeConfPose ?r ?q ?s2 ?p2)
    (EqValPosePose ?p1 ?p2)

    (AtPose ?s ?p)
    (AtConf ?r ?q)
    (AttachTo ?s ?a)
    (AtBinding ?s ?b)
    (Active ?r)
    (Equal ?q1 ?q2)
    (CanMove)

    (On ?s ?a)
    (Bound ?s ?a)
    (UnsafePose ?s ?p)
    (UnsafeConf ?r ?q)
    (FindConf ?p ?s ?a)
    (MoveToConf ?r ?p ?s ?a)
    (GoToConf ?r ?p ?s ?a)
    (FindBinding ?a ?s)
    (FindBP ?s ?a)
  )

  (:action motion
    :parameters (?r ?t ?q1 ?q2)
    :precondition (and (Motion ?r ?t ?q1 ?q2)
                       (AtConf ?r ?q1) (CanMove)
                       (not (= ?q1 ?q2))
                  )
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)) (not (CanMove)))
  )
  (:action motion_holding
    :parameters (?s ?a ?b ?p1 ?p2 ?r ?t ?q1 ?q2)
    :precondition (and (Motion ?r ?t ?q1 ?q2)
                       (BindingPose ?s ?a ?b ?p1)
                       (Kin ?s ?p1 ?a ?b ?r ?q1)
                       (BindingPose ?s ?a ?b ?p2)
                       (Kin ?s ?p2 ?a ?b ?r ?q2)
                       (AtConf ?r ?q1)
                       (AtPose ?s ?p1) (CanMove)
                       (not (= ?q1 ?q2))
                  )
    :effect (and (AtConf ?r ?q2) (AtPose ?s ?p2)
                 (not (AtConf ?r ?q1)) (not (AtPose ?s ?p2)) (not (CanMove)))
  )
  (:action attach
    :parameters (?s ?p1 ?p2 ?a1 ?b1 ?a2 ?b2 ?r ?q)
    :precondition (and (Kin ?s ?p1 ?a2 ?b2 ?r ?q) (BindingPose ?s ?a2 ?b2 ?p2) (AtPose ?s ?p1) (BindingPose ?s ?a1 ?b1 ?p1) (AtConf ?r ?q) (Active ?r)
                       (not (AtPose ?s ?p2))
                       (not (UnsafePose ?s ?p2))
                       (not (UnsafeConf ?r ?q))
                       (not (CanMove))
                       (not (= ?a1 ?a2))
                  )
    :effect (and (AtBinding ?s ?b2)
                 (CanMove)
                 (not (AtPose ?s ?p1))
            )
  )
  (:action detach
    :parameters (?s ?p1 ?p2 ?a1 ?b1 ?a2 ?b2 ?r ?q)
    :precondition (and (Kin ?s ?p2 ?a1 ?b1 ?r ?q) (BindingPose ?s ?a1 ?b1 ?p1) (AtBinding ?s ?b1) (BindingPose ?s ?a2 ?b2 ?p2) (AtConf ?r ?q) (Active ?r)
                       (not (UnsafePose ?s ?p2))
                       (not (UnsafeConf ?r ?q))
                       (not (CanMove))
                  )
    :effect (and (AtPose ?s ?p2)
                 (CanMove)
                 (not (AtBinding ?s ?b1))
            )
  )

  (:derived (UnsafePose ?s ?p)
    (exists (?s2 ?p2) (and (Pose ?s ?p) (Pose ?s2 ?p2) (not (= ?s ?s2))
                           (not (CFreePosePose ?s ?p ?s2 ?p2))
                           (AtPose ?s2 ?p2)))
  )
  (:derived (UnsafeConf ?r ?q)
    (exists (?s2 ?p2) (and (Conf ?r ?q) (Pose ?s2 ?p2)
                           (not (CFreeConfPose ?r ?q ?s2 ?p2))
                           (AtPose ?s2 ?p2)))
  )
  (:derived (On ?s ?a)
    (exists (?p ?b) (and (BindingPose ?s ?a ?b ?p) (AtPose ?s ?p)))
  )
  (:derived (Bound ?s ?a)
    (exists (?p ?b) (and (BindingPose ?s ?a ?b ?p) (AtBinding ?s ?b)))
  )
  (:derived (FindConf ?p ?s ?a)
    (exists (?r ?q ?b) (and (Kin ?s ?p ?a ?b ?r ?q) (Conf ?r ?q)))
  )
  (:derived (GoToConf ?r ?p ?s ?a)
    (exists (?q) (and (FindConf ?p ?s ?a) (AtConf ?r ?q)))
  )
  (:derived (MoveToConf ?r ?p ?s ?a)
    (exists (?q ?b) (and (Kin ?s ?p ?a ?b ?r ?q) (AtConf ?r ?q)))
  )
  (:derived (FindBinding ?a ?s)
    (exists (?b) (Binding ?s ?a ?b))
  )
  (:derived (FindBP ?s ?a)
    (exists (?b ?p) (BindingPose ?s ?a ?b ?p))
  )
)