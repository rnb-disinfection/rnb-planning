## Check List

### 0. Installation

### 1. Robot
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

### 2. Sensor
#### 2.1 Kinect
* get_image()

#### 2.2 Realsense
* get_image()

#### 2.3 ArucoStereo
* calibrate()
* detect()
  * level_mask
  * name_mask
  * visualize
* add_item_axis()

### 3. SceneBuilder, GeometryHandle
#### 3.1 SceneBuilder
* detect_items
* create_gscene
* detect_and_register
* add_poles
* add_robot_geometries

#### 3.2 GeometryHandle
* create_safe
  * BOX, CYLINDER, CAPSULE, SPHERE, ARROW

### 4. PlanningPipeline
#### 4.1 PlanningScene
* create_binder
* create_object
* update_state

#### 4.2 MotionPlanner
* MoveitPlanner
* EtaslPlanner

#### 4.3 MotionFilter
* GraspFilter
* ReachFilter
* LatticedFilter

#### 4.4 TaskPlanner
* RRT
* Sampler

#### 4.5 Execution
* play_schedule
* execute_schedule

### 5. Web UI
#### 5.1 Geometry
* highlight
* add/delete
* move
* apply
* save/load

#### 5.2 Binding
* highlight
* move
* apply
* save/load

#### 5.3 Mark
* highlight
* MarkEnv
* MarkObj

#### 5.4 Plan
* MotionPlanner
* TaskPlanner
  * Plan
* PlanList
* Replay
* Execute
##### TaskPlanner - specify goal
##### Save/Load

#### 5.4 Setting - TBD

### 6. Custom Tasks
#### 6.1 Sweep Task
* script

### 7. Demo
#### 7.1 Multi Object Handing
* script

#### 7.2 White board sweeping
* script