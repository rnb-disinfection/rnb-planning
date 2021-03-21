# RNB-Planning v0.1.0 Release Notes (2021-3-20)
## New features
### 1. Robots
* **1.1. indy**  
  - joint_move_make_sure, move_joint_s_curve, move_joint_wp, grasp  
* **1.2. panda**  
  - joint_move_make_sure,  move_joint_s_curve,  move_joint_wp,  grasp  
* **1.3. combined_robot**  
  - joint_move_make_sure, move_joint_wp, grasp  

### 2. Sensors
* **2.1 Kinect**  
  - initialize, get_config, get_image, disconnect  
* **2.2 Realsense**  
  - initialize, get_config, get_image, disconnect  
* **2.3 ArucoStereo**  
  - initialize, calibrate, detect, disconnect

### 3. SceneBuilder, GeometryHandle
* **3.1 SceneBuilder**  
  - reset_reference_coord, detect_items, create_gscene, add_robot_geometries, add_poles, detect_and_register
* **3.2 GeometryHandle**  
  - create_safe
    - BOX, CYLINDER, PLANE, CAPSULE, SPHERE, ARROW
  - show_pose, show_motion

### 4. PlanningPipeline
*\* ~~Strikethrough~~ means no-error but none-functional cases.*
* **4.1 PlanningScene**  
  - create_binder
    - actor: PlacePlane, Gripper2Tool, SweepTool
    - handle: Grasp2Point, PlacePoint, SweepPoint
  - create_subject
    - subject: CustomObject, SweepTask
  - update_state
* **4.2 MotionPlanner**  
  - MoveitPlanner
    - pick, place, sweep
  - EtaslPlanner
    - pick, place, ~~sweep~~
* **4.3 TaskPlanner**  
  - RRT
    - pick & place & sweep
      - single process
      - multi process
    
### 4-A MotionFilter
*\* ~~Strikethrough~~ means no-error but none-functional cases.*
* **4-A.1 GraspChecker**  
  - pick, place, sweep
* **4-A.2 ReachChecker**  
  - pick, place, ~~sweep~~
* **4-A.3 LatticedChecker**  
  - pick, place, ~~sweep~~

### 5 Planning & Execution
 - make test function (real world single object P&P)
* **5.1 Indy**
* **5.2 Panda**
* **5.3 Indy&Panda**

### 6. Web UI
* **6.1 Geometry**  
  - highlight, add/delete, move, apply, save/load
* **6.2 Binding**  
  - highlight, move, apply, save/load
* **6.3 Mark**  
  - highlight
  - MarkEnv, MarkObj
* **6.4 Plan**  
  - MotionPlanner
  - TaskPlanner
    - Plan
  - PlanList
    - Replay, Execute
* **6.5 Setting - TBD**  
##### TaskPlanner - specify goal
##### Save/Load - deactivate

### 7. Demo
* **7.1 Multi Object Handing**  
  - script
* **7.2 Using Tool to sweep**  
  - script
* **7.3 White board sweeping**  
  - script
  
  
## Deprecated features
* **None (First release)**  
  
  
## Requirements

### 1. Robot controller version
  - [rnb-control v0.2.0](https://github.com/rnb-disinfection/rnb-control/releases/tag/v0.2.0-panda)

### 2. Model
* **ReachChecker**
  - model/reach_svm/indy7.json
  - model/reach_svm/panda.json
  - 백업: 개인 이동식 하드디스크, 강준수
* **LatticedChecker**
  - model/latticized/indy7/20210222-134724
  - model/latticized/panda/20210222-145610
  - 백업: 개인 이동식 하드디스크, 강준수
