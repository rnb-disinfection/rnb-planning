# Panda simulator
* Install
  ```
  pip install numpy numpy-quaternion rospy-message-converter==0.4.0 \
  && cd ~/catkin_ws && sudo rm -rf devel build \
  && cd ~/catkin_ws/src && git clone https://github.com/justagist/panda_simulator && cd panda_simulator && ./build_ws.sh  
  ```
* Launch
  ```
  roslaunch panda_gazebo panda_world.launch start_moveit:=false   
  ```

# Launch Indy simulator (CadKit)
* run TaskManager (simulator mode)  on STEP
  ```
  cd /home/user/release/IndyFramework3.0 && ./TaskManager -j indyDeploy.json
  ```
* open cadkit (NRMK Launcher -> CadKit
  * Insert Indy7 model ("C:\Program Files (x86)\neuromeka\NRMKFoundation\bin\models\Indy7\Model_Indy7.wrl")
  * Connect -> Enter ip of STEP PC -> Connect

  
# How to make xacro for multi-robot  
* find xacro file in the description package for target robot  
* copy the xacro file to "$TAMP_ETASL_DIR"/robots  
* delete "world" link and joint connected to it  
* add macro:  
  ```
  \<xacro:macro name="robotname" params="robot_id:='0' description_pkg:='robot_description' connected_to:='' xyz:='0 0 0' rpy:='0 0 0'"\>  
  \<xacro:unless value="${not connected_to}"\>  
  \<joint name="robotname${robot_id}\_joint\_${connected_to}" type="fixed"\>  
  \<parent link="${connected_to}"/\>  
  \<child link="robotname${robot_id}_link0"/\>  
  \<origin rpy="${rpy}" xyz="${xyz}"/\>  
   \</joint\>  
  \</xacro:unless\>  
  \<!-- robot contents  --\>  
  \<!-- robot contents  --\>  
  \<!-- robot contents  --\>  
  \</xacro:macro\>  
  ```
* change all item names: selectively replace {name="} with {"name="robotname${robot_id}}  
* include and call the xacro file in "custom_robots.urdf.xacro"  
* test generating URDF file  
  ```
  rosrun xacro xacro "$TAMP_ETASL_DIR"robots/custom_robots.urdf.xacro \> "$TAMP_ETASL_DIR"robots/custom_robots.urdf  
  ```
* run rviz  
  ```
  roslaunch "$TAMP_ETASL_DIR"/launch/gui_custom_robots_joint_panel.launch  
  ```
  * for "not unique" error, remove it from individual xacro files and include the item on the top of "custom_robots.urdf.xacro"  


# Deprecated dependencies  

## python packages  
```
pip install klampt
```

## OMPL  
```
cd ~/Projects/tamp_etasl/ompl
chmod +x *
./install-ompl-ubuntu.sh --python
```