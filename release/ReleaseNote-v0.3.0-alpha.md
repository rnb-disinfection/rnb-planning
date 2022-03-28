# RNB-Planning v0.3.0-alpha Release Notes (2022-03-29)
  
## Preparing features

### IncrementalSearch
* Incremental TAMP by recursive constraint solving (src/pkg/planning/incremental_search.py)
* Contact: Junsu Kang, junsu_kang@postech.ac.kr
  
### PushObject Task
*  **SlidePoint** action point, **PushObject** task are added to implement pushing tasks
* Contact: Junsu Kang, junsu_kang@postech.ac.kr

### NonHolonomicPlanner
* Simple python implementation of nonholonomic planar mobile robot planning (src/scripts/developing/PushContact/nonholonomic_planner.py)
* Contact: Junsu Kang, junsu_kang@postech.ac.kr

### Postech Mobile Robot
* **PostechMobileClient**
  - Communication client for RNB Mobile Robot in **rnb-control v0.5.0**
  - Corresponds to ***RobotType.pmb***
* Contact: Junsu Kang, junsu_kang@postech.ac.kr


## Updates

### Grouping Actor-Actee by key
  - Valid pairs of *actors* and *actees* can be grouped by setting key now. Instruction in release/4.PlanningPipeline.ipynb
  
### Custom Planner
  - *custom_planner* and *custom_planner_joint* features are added in MoveitPlanner

###  MultiICP updates
  - To be filled by jhk
  
    
## Known issues
* **WebUI **
  * 양쪽 창에 같은 탭을 띄울 경우 마지막에 띄운 탭의 버튼만 동작함.
* **Setup**
  * moveit_interface_py 빌드 전 apt update && sudo apt-get dist-upgrade 필요 - repository 등 설정 꼬여있던 부분 문제일 수 있음, 확인 필요.
* **BUG**
  * GraspChecker - sphere에 대한 collision checking 이상
  
  
## Requirements

### 1. Robot controller version
  - [rnb-control v0.5.0](https://github.com/rnb-disinfection/rnb-control/releases/tag/v0.5.0)

### 2. Model
* **ReachChecker**
* To use **ReachChecker**, get model files from https://github.com/rnb-disinfection/arxiv-highcap/tree/rnb-planning/model/reach_svm and put them in *$RNB_PLANNING_DIR/model/reach_svm* folder.
* For robots that are indy7 variants (indy7gripper...), copy indy7 model file and assign the robot name as the file name.
* **LatticedChecker**
* To use **LatticedChecker**, get model files from https://github.com/rnb-disinfection/arxiv-highcap/tree/rnb-planning/model/latticized and put them in *$RNB_PLANNING_DIR/model/latticized* folder.
* For robots that are indy7 variants (indy7gripper...), copy indy7 model file and assign the robot name as the file name.
