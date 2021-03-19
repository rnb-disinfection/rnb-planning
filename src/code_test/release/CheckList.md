## Check List

### 0. Installation

### 1. Robot
#### 1.1. indy
* joint_make_sure()
* move_joint_s_curve()
* move_joint_wp()
* grasp()

#### 1.2. panda
* joint_make_sure()
* move_joint_s_curve()
* move_joint_wp()
* grasp()

#### 1.3. combined_robot
* joint_make_sure()
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

### 4. PlanningScene

### 5. PlanningPipeline

### 6. Web UI

### 7. Custom Tasks
#### 7.1 Sweep Task


### 8. Demo
#### 8.1 White board sweeping

