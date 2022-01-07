# RNB-Planning v0.2.0 Release Notes (2021-11-18)

## New features
#### Detector
* MultiICP
  - Implement as Detector subclass
    - Detection rule using mmdet & MultiICP 
  - Need to add usage script
* CombinedDetector
  - Camera class as singletone
  
### 4. PlanningPipeline
* **4.1 PlanningScene**  
  - **create_binder()**
    - *point* argument must be passed to ***pscene.create_binder()*** now. pass point=None if error occurs.
  - **BindingTransform**
    - ***pscene.sample_leaf_state()*** now returns *to_state* only. *redundancy_dict* is contained in ***to_state.binding_state***.
    - ***State.binding_state*** is a dictionary of **BindingTransform**. The redundancy can be accessed at ***BindingTransform.redundancy***.
    - To adjust redundancy, call ***BindingTransform.set_redundancy()*** and ***BindingTransform.update_transforms()***.
* **4.2 MotionPlanner**  
  - **MoveitPlanner**
    - Interpolated path generation by incremental IK
* **4.3 TaskPlanner**  
  - **PDDLStream (Pick&Place)**  
  
## Deprecated features
#### eTaSL planner
  
## Known issues
* **WebUI **
  * 양쪽 창에 같은 탭을 띄울 경우 마지막에 띄운 탭의 버튼만 동작함.
* **Setup**
  * moveit_interface_py 빌드 전 apt update && sudo apt-get dist-upgrade 필요 - repository 등 설정 꼬여있던 부분 문제일 수 있음, 확인 필요.
* **BUG**
  * GraspChecker - sphere에 대한 collision checking 이상
  
  
## Requirements

### 1. Robot controller version
  - [rnb-control v0.4.0](https://github.com/rnb-disinfection/rnb-control/releases/tag/v0.4.0)

### 2. Model
* **ReachChecker**
* To use **ReachChecker**, get model files from https://github.com/rnb-disinfection/arxiv-highcap/tree/rnb-planning/model/reach_svm and put them in *$RNB_PLANNING_DIR/model/reach_svm* folder.
* For robots that are indy7 variants (indy7gripper...), copy indy7 model file and assign the robot name as the file name.
* **LatticedChecker**
* To use **LatticedChecker**, get model files from https://github.com/rnb-disinfection/arxiv-highcap/tree/rnb-planning/model/latticized and put them in *$RNB_PLANNING_DIR/model/latticized* folder.
* For robots that are indy7 variants (indy7gripper...), copy indy7 model file and assign the robot name as the file name.
