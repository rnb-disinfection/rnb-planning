## TODO
  - Constraint 추가
  - State space를 ConstrainedStateSpace로 변경
  - Validity checker를 ConstrainedState에 맞춰 수정

### class ModelBasedPlanningContext : public planning_interface::PlanningContext
  - Representation of a particular planning context -- the planning scene and the request are known, solution is not yet computed.
  - solve() 포함

### OMPLInterface
  - OMPL-Moveit 인터페이스

### PlanningContextManager:
  - getPlanningContext / getPlannerConfigureations / setPlannerConfigureations 셔틀
  - scene에서 PlannerContext 생성하는 역할, OMPLInterface에서만 사용

### class OMPLPlannerManagerCustom : public planning_interface::PlannerManager
  - Base class for a MoveIt planner.
  - 실제 플래닝 외부와 연결 
  ```
  initialize(robot_model_, __node_handle->getNamespace())
  getPlanningContext(planning_scene_, _req, _res.error_code, custom_constraint)
  ```
  
  
#### ConstrainedStateSpace::StateType
  * WrapperStateSpace::StateType, VectorXd 상속
  * 생성자에서 getValueAddressAtIndex으로 VectorXd 데이터 internal state와 data array 동기화
  * copy로 다른 VectorXd에서 카피 가능, internal state와는 data array만 동기화됨.
  
#### ModelBasedStateSpace::StateType
  * 스테이트는 double * values에 저장, tag, flags, distance 등 멤버 변수 추가
  * state_validity_checker, constraints_library, projection_evaluators에서만 사용 (ModelBasedStateSpace::StateType로 검색)
  
  