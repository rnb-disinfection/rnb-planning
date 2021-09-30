# SUPPLEMENT
## Recommended Tools  
* jupyter 
  * install jupyter  
  ```bash
  sudo apt install python3-notebook python-notebook jupyter jupyter-core python-ipykernel  
  ```
  * do server setting as follows
  ```bash
  jupyter notebook --generate-config \
  && jupyter notebook password \
  && vi ~/.jupyter/jupyter_notebook_config.py
  ```
  * enter password twice
  * find #c.NotebookApp.ip = 'localhost'
  * remove '#' and replace 'localhost' with 'your_ip'
  * find and set c.NotebookApp.open_browser = False if you don't want to open browser when starting jupyter
  
  
* nvidia-smi-gui
  * install  
  ```bash
  sudo apt-get install python3-pyqt4 \
  && mkdir ~/nvidia-smi-gui && cd ~/nvidia-smi-gui \
  && git clone https://github.com/imkzh/nvidia-smi-gui.git .
  ```
  * launch  
  ```bash
  python3 ~/nvidia-smi-gui/nvidia-smi-gui.py
  ```
* Anydesk - enable unattended access in Settings-Security
* GitKraken  
* PyCharm, Clion  
  * add "export PATH=$PATH:{}/bin" to .bashrc  
* openssh-server  
  ```bash
  sudo apt-get install openssh-server -y && sudo service ssh start
  ```

* Doxygen
  * Install
  ```bash
  sudo apt-get install -y doxygen doxygen-gui texlive-latex-base texlive-latex-recommended ko.tex texlive-fonts-extra
  ```

  * Run
  ```bash
  doxywizard
  ```
  
* ikfast (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html) - cannot install openrave


## TIPS 
* Launching RVIZ
  ```bash
  roslaunch "$RNB_PLANNING_DIR"src/launch/gui_custom_robots_joint_panel.launch 
  ``` 

* Launch franka ros interface  
  * visualization:
  ```bash
  roslaunch franka_visualization franka_visualization.launch robot_ip:=192.168.0.13 load_gripper:=true  
  ```
  * launch interface: 
  ```bash
  roslaunch franka_interface interface.launch
  ```
* starting roscore if it's not active  
  ```bash
  nohup roscore &  
  ```
* to reset node/topic/params, kill and restart roscore  
  ```bash
  sudo killall -9 roscore && nohup roscore &  
  ```
* ROS keyserver error? When installing new ros package and apt-update are impossible: get ros2 key?
  ```
  curl http://repo.ros2.org/repos.key | sudo apt-key add -
  ```
* Killing zombie network process
  ```bash
  netstat -lntp
  kill -9 <PID>
  ```


## Panda simulator
* Install
  ```bash
  pip install numpy numpy-quaternion rospy-message-converter==0.4.0 \
  && cd ~/catkin_ws && sudo rm -rf devel build \
  && cd ~/catkin_ws/src && git clone https://github.com/justagist/panda_simulator && cd panda_simulator && ./build_ws.sh  
  ```
* Launch
  ```bash
  roslaunch panda_gazebo panda_world.launch start_moveit:=false   
  ```


## Launch Indy simulator (CadKit)
* run TaskManager (simulator mode)  on STEP
  ```bash
  cd /home/user/release/IndyFramework3.0 && ./TaskManager -j indyDeploy.json
  ```
* open cadkit (NRMK Launcher -> CadKit
  * Insert Indy7 model ("C:\Program Files (x86)\neuromeka\NRMKFoundation\bin\models\Indy7\Model_Indy7.wrl")
  * Connect -> Enter ip of STEP PC -> Connect

  
## How to make xacro for multi-robot  
* find xacro file in the description package for target robot  
* copy the xacro file to "$RNB_PLANNING_DIR"/src/robots  
* delete "world" and "base_link" links and joints connected to it  
* add below macro on the top of the file, replace \<robotname\> and \<root_link\> to your robot name and the root link name.  
  ```xml
  <xacro:macro name="<robotname>" params="robot_id:='0' description_pkg:='robot_description' connected_to:='' xyz:='0 0 0' rpy:='0 0 0'">  
  <xacro:unless value="${not connected_to}">  
  <joint name="<robotname>${robot_id}_joint_${connected_to}" type="fixed">  
  <parent link="${connected_to}"/>  
  <child link="<robotname>${robot_id}_<root_link>"/>  
  <origin rpy="${rpy}" xyz="${xyz}"/>  
  </joint>  
  </xacro:unless>  
  <!-- robot contents  -->  
  <!-- robot contents  -->  
  <!-- robot contents  -->  
  </xacro:macro>  
  ```
* change all link and joint names to have this form: "\<robotname\>${robot_id}\_\<itemname\>" 
* include and call the xacro file in "custom_robots_src.urdf.xacro"  
* test generating URDF file  
  ```bash
  rosrun xacro xacro "$RNB_PLANNING_DIR"src/robots/custom_robots.urdf.xacro > "$RNB_PLANNING_DIR"src/robots/custom_robots.urdf  
  ```
* run rviz  
  ```bash
  roslaunch "$RNB_PLANNING_DIR"/src/launch/gui_custom_robots_joint_panel.launch  
  ```
  * for "not unique" error, remove it from individual xacro files and include the item on the top of "custom_robots.urdf.xacro"  

## Removing cuda  
  ```bash
  sudo apt remove -y 'cuda*'
  sudo apt remove -y 'libcuda*'
  sudo apt remove -y 'cudnn*'
  sudo apt remove -y 'libcudnn*'
  ```

## Deprecated dependencies  

### python packages  
```bash
pip install klampt
```

### STOMP-ROS  
```bash
cd $HOME/catkin_ws/src \
&& git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel \
&& git clone https://github.com/ros-industrial/stomp_ros.git -b melodic-devel \
&& cd $HOME/catkin_ws \
&& catkin_make -DCMAKE_BUILD_TYPE=Release  
```  

### Tesseract  
* clone tesseract on workspace  
  ```bash
  cd ~/Projects/rnb-planning/ws_ros/src \
  && git clone https://github.com/ros-industrial-consortium/tesseract.git  
  ```
   
* clone all dependencies in dependency.rosinstall  
  ```bash
  git clone https://github.com/ros-industrial/cmake_common_scripts.git cmake_common_scripts \
  && git clone https://github.com/ros-industrial-consortium/tesseract_ext.git tesseract_ext \
  && git clone https://github.com/ros-industrial-consortium/trajopt_ros.git trajopt \
  && git clone https://github.com/swri-robotics/descartes_light.git  descartes_light \
  && git clone https://github.com/Jmeyer1292/opw_kinematics.git opw_kinematics \
  && git clone https://github.com/ethz-adrl/ifopt.git ifopt  
  ```
  * check if any other dependency is added on recent version.
  
  
* install base dependency
  ```bash
  sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev \
  && sudo apt install lcov \
  && sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
  && sudo apt-get install bison  
  ```

* build workspace
  ```bash
  cd .. && sudo rm -rf devel && sudo rm -rf build
  catkin build --force-cmake -DTESSERACT_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  && add export TESSERACT_SUPPORT_DIR='/home/tamp/Projects/rnb-planning/ws_ros/devel/share/tesseract_support'  
  ```
  
* source workspace (done in project setup section)
  ```bash
  source ~Projects/rnb-planning/ws_ros/devel/setup.bash
  ``` 
  
### Moveit
* Install moveit (after ros-melodic and catkin)
  ```bash
  sudo apt install ros-melodic-moveit
  ```

* Tutorial
  * make workspace and clone tutorial repository
  ```bash
  mkdir -p ~/ws_moveit/src
  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
  git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
  ```
  * install packages
  ```bash
  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro melodic
  cd ~/ws_moveit
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build
  ```
  * source workspace
  ```bash
  source ~/ws_moveit/devel/setup.bash
  ```
  * follow tutorial process in http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started
