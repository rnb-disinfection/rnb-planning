# Constrained planning
## Moveit low-level
* planning_pipeline 외에는 low_level, 활용 가능 (node binding 없음)
* generatePlan 집중 분석
* adapter_chain은 부가 작업, 필요 X
* displayComputedMotionPlans, checkSoutionPaths 도 불필요한 것으로 판단, 미도입
* model_based_planning_context nh 참조 / 미사용 <loadConstraintApproximations - constraint_approximation_path 없음>
* model_based_planning_context: nh 참조 / 미사용 -- configure -> loadConstraintApproximations -> constraint_approximation_path
  
* planning_context_manager: nh 참조
   -- getPlanningContext(nh) -> configure(nh, 미사용)
   -- getPlanningContext(): 미사용됨
  
* ompl_interface _nh 멤버 -> getPlanningContext(nh)
  
* planner_interface는 nh 기본 멤버로 가짐, 이 부분 돌아가야 함.
 - 이 부분의 실제 사용은 getPlannerContext -> context.solve 뿐임.
* getPlanningContext는 context_manager_.getPlanningContext -> context->simlifySolutions(할당자) 의 미러
  
* ompl::StateValidityChecker의 isValid 오버라이드 하고 아래 함수 호출해 validity check 함.
  * ompl::SpaceInformation::satisfiesBounds
  * moveit_core::KinematicConstraintSet::decide
  * ModelBasedPlanningContext::PlanningScene::isStateFeasible
  * ModelBasedPlanningContext::PlanningScene::checkCollision