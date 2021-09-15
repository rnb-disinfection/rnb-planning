(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Stackable ?o ?r)

    (Conf ?q)
    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?o ?p ?g ?q ?t)
    (FreeMotion ?q1 ?t ?q2)
    (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    (Supported ?o ?p ?r)
    (Traj ?t)

    (TrajCollision ?t ?o2 ?p2)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)

    (AtPose ?o ?p)
    (AtGrasp ?o ?g)
    (Graspable ?o)
    (HandEmpty)
    (AtConf ?q)
    (CanMove)
    
    (On ?o ?r)

    (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeTraj ?t)
  )

  (:action move_free
    :parameters (?q1 ?q2 ?t)
    :precondition (and (FreeMotion ?q1 ?t ?q2)
                       (AtConf ?q1) (HandEmpty) (CanMove)
                       ;(not (UnsafeTraj ?t))
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )
  (:action move_holding
    :parameters (?q1 ?q2 ?o ?g ?t)
    :precondition (and (HoldingMotion ?q1 ?t ?q2 ?o ?g)
                       (AtConf ?q1) (AtGrasp ?o ?g) (CanMove)
                       ;(not (UnsafeTraj ?t))
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )

  (:action pick
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t) (not (CanMove))
                       (AtPose ?o ?p) (HandEmpty) (AtConf ?q)
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeTraj ?t))
                  )
    :effect (and (AtGrasp ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty)))
  )
  (:action place
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t) (not (CanMove))
                       (AtGrasp ?o ?g) (AtConf ?q)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeTraj ?t))
                  )
    :effect (and (AtPose ?o ?p) (HandEmpty) (CanMove)
                 (not (AtGrasp ?o ?g)))
  )
  
  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )

  (:derived (UnsafePose ?o ?p)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreePosePose ?o ?p ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeApproach ?o ?p ?g)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Grasp ?o ?g) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreeApproachPose ?o ?p ?g ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeTraj ?t)
    (exists (?o2 ?p2) (and (Traj ?t) (Pose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
)