## Check List

### 0. Installation


### 1. Robots

#### 1.1. indy
* joint_move_make_sure()
* move_joint_s_curve()
* move_joint_wp()
* grasp()

#### 1.2. panda
* joint_move_make_sure()
* move_joint_s_curve()
* move_joint_wp()
* grasp()

#### 1.3. combined_robot
* joint_move_make_sure()
* move_joint_wp()
* grasp()


### 2. Sensors

#### 2.1 Kinect
* initialize()
* get_config()
* get_image()
* disconnect()

#### 2.2 Realsense
* initialize()
* get_config()
* get_image()
* disconnect()

#### 2.3 ArucoStereo
* initialize()
* calibrate()
* detect()
  * level_mask
  * name_mask
  * visualize
* disconnect()


### 3. SceneBuilder, GeometryHandle

#### 3.1 SceneBuilder
* reset_reference_coord
* detect_items
* create_gscene
* add_robot_geometries
* add_poles
* detect_and_register

#### 3.2 GeometryHandle
* create_safe
  * BOX, CYLINDER, PLANE, CAPSULE, SPHERE, ARROW
* show_pose
* show_motion


### 4. PlanningPipeline - virtual

#### 4.1 PlanningScene
* create_binder
* create_object
* update_state

#### 4.2 MotionPlanner
* MoveitPlanner
  - pick, place, sweep
* EtaslPlanner
  - pick, place, sweep

#### 4.3 MotionFilter
* GraspFilter
  - pick, place, sweep
* ReachFilter
  - pick, place, sweep
* LatticedFilter
  - pick, place, sweep

#### 4.4 TaskPlanner
* RRT
  - pick & place, sweep, combined


### 5 Planning & Execution - make test function (real world single object P&P)
* Indy
* Panda
* Indy&Panda


### 6. Web UI

#### 6.1 Geometry
* highlight
* add/delete
* move
* apply
* save/load

#### 6.2 Binding
* highlight
* move
* apply
* save/load

#### 6.3 Mark
* highlight
* MarkEnv
* MarkObj

#### 6.4 Plan
* MotionPlanner
* TaskPlanner
  * Plan
* PlanList
* Replay
* Execute
##### TaskPlanner - specify goal
##### Save/Load - deactivate

#### 6.5 Setting - TBD


### 7. Demo

#### 7.1 Multi Object Handing
* script

#### 7.2 White board sweeping
* script
