# Requirements  
* Ubuntu 18.04  
* Nvidia driver 440  
  * sudo add-apt-repository ppa:graphics-drivers/ppa  
  * sudo apt update  
  * sudo apt-get install nvidia-driver-440  
  * sudo reboot  
* cuda 10.1 (add PATH, LD_LIBRARY_PATH)  
* cudnn 7.6  
* pip3 (apt-get install python3-pip && pip3 install --upgrade pip)  
* tensorflow 2.3.0  
* ROS Melodic  
  * wget https://raw.githubusercontent.com/orocapangyo/meetup/master/190830/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh  
  * sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
  * sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116  
  * sudo apt-get update && sudo apt-get upgrade -y  
  * sudo apt-get install ros-melodic-desktop-full  
  * sudo apt-get install ros-melodic-rqt*  
  * sudo apt-get install python-rosdep  
  * sudo rosdep init  
  * rosdep update  
  * sudo apt-get install python-rosinstall  
* Moveit  
  * sudo apt-get install ros-melodic-moveit ros-melodic-industrial-core ros-melodic-moveit-visual-tools ros-melodic-joint-state-publisher-gui  
* Gazebo  
  * sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-trajectory-controller  
* UR package  
  * link: https://github.com/ros-industrial/universal_robot  
  * cd $HOME/catkin_ws/src  
  * git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git  
  * cd $HOME/catkin_ws  
  * rosdep update  
  * rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src  
  * catkin_make -DCMAKE_BUILD_TYPE=Release  
* Indy package
  * cd ~/catkin_ws/src && git clone -b  release-2.3 https://github.com/neuromeka-robotics/indy-ros
  * cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
  * Follow instruction on IndyFramework3.0/ReadMe.md to update Indy robot to 3.0
* Franka package  
  * sudo apt install ros-melodic-libfranka ros-melodic-franka-ros  
  * Copy/move the franka.sh file to the root of the catkin_ws $ cp ~/catkin_ws/src/franka_ros_interface/franka.sh ~/catkin_ws/
  * Change the values in the copied file (described in the file).

* panda simulator  
  * pip install numpy numpy-quaternion rospy-message-converter==0.4.0  
  * cd ~/catkin_ws && sudo rm -rf devel build  
  * cd ~/catkin_ws/src && git clone https://github.com/justagist/panda_simulator && cd panda_simulator && ./build_ws.sh  
* eTaSL  
  * install gcc7: sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-7 g++-7 gcc-7-multilib g++-7-multilib  
  * install gcc5: sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-5 g++-5 gcc-5-multilib g++-5-multilib  
  * set gcc alternative versions:  
    * sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20  
    * sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40  
    * sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20  
    * sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 40  
  * check gcc/g++ alternative version  
    * sudo update-alternatives --display gcc && sudo update-alternatives --display g++  
  * switch to recommended  gcc/g++ version for each install in below link  
    * sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  * https://etasl.pages.gitlab.kuleuven.be/install-new.html  
  * (add "source $OROCOS/install_isolated/setup.bash" on top of user section of "~/.bashrc")  
  * overwrite custom etasl project from github and recompile etasl
  * in "\~/etasl_ws", "\~/etasl_ws/etasl/ws/etasl", "\~/etasl_ws/etasl/ws/etasl-py"  
  * sudo rm -rf devel && sudo rm -rf devel && catkin_make -DCMAKE_BUILD_TYPE=Release  
  * source /home/junsu/etasl_ws/etasl/ws/etasl-py/devel/setup.bash   
  * **if eTaSL simulation is slow (has to be under 300ms with 200 constraints), re-compile packages in release mode**  
* Azure Kinect  
  * setup microsoft repository  
    * curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -  
    * sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod  
    * sudo apt-get update  
  * install sdk  
    * sudo apt install libk4a1.4  
    * sudo apt install libk4a1.4-dev  
    * sudo apt install k4a-tools  
  * allow non-root usage  
    * Copy 'azure/99-k4a.rules' into '/etc/udev/rules.d/'.  
    * Detach and reattach Azure Kinect devices if attached during this process.  
  * install open3d  
    * pip install open3d  
* Realsense
  * instruction in https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md  
  * install pyrealsense2  
    * pip install pyrealsense2  
  
  
## Install Tesseract  
  
* clone tesseract on workspace  
  * cd ˜/Projects/tf_gmt/eTaSL/ws_ros  
  * git clone https://github.com/ros-industrial-consortium/tesseract.git  
   
* clone all dependencies in dependency.rosinstall  
  * git clone https://github.com/ros-industrial/cmake_common_scripts.git cmake_common_scripts  
  * git clone https://github.com/ros-industrial-consortium/tesseract_ext.git tesseract_ext  
  * git clone https://github.com/ros-industrial-consortium/trajopt_ros.git trajopt  
  * git clone https://github.com/swri-robotics/descartes_light.git  descartes_light  
  * git clone https://github.com/Jmeyer1292/opw_kinematics.git opw_kinematics  
  * git clone https://github.com/ethz-adrl/ifopt.git ifopt  
  * check if any other dependency is added on recent version.
  
* install base dependency
  * sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev  
  * sudo apt install lcov  
  * sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev  

* build workspace
  * catkin build --force-cmake -DTESSERACT_ENABLE_TESTING=ON 
    (-DCMAKE_BUILD_TYPE=Release is set by default)
  * add export TESSERACT_SUPPORT_DIR='/home/junsu/Projects/tf_gmt/eTaSL/ws_ros/devel/share/tesseract_support'  
  
  
  
# Python packages
* ros compatibility  
  * pip install rospkg  
* numba  
  * pip install colorama==0.3.9 llvmlite==0.31.0 numba==0.47.0  
* misc.  
  * pip install matplotlib trimesh pathlib protobuf grpcio numpy-stl sklearn filterpy  
* pymanopt
  * pip install autograd && pip install --user pymanopt==0.2.4

# Setup project  
* rebuild custom workspace  
  * cd eTaSL/ws_ros && rm -rf build devel && catkin_make -DCMAKE_BUILD_TYPE=Release  
* add below to ~./bashrc  
  * export TF_GMT_ETASL_DIR=/home/junsu/Projects/tf_gmt/eTaSL/  
  * source "$TF_GMT_ETASL_DIR"ws_ros/devel/setup.bash  
  * start roscore if it's not on  
    * nohup roscore &  

# Setup and launch panda repeater
* setup panda_ros_repeater on panda master pc (https://github.com/Cucumberkjs/panda_ros_repeater.git)  
* launch panda command repeater on matser  
  * roslaunch panda_ros_repeater joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=false

# Setup and launch indy online tracker
* Download and install NRMK IndyFramework and PlatformSDK (framework 2.3.1 -> Platform 3.0.5)  
* Clone IndyFramework2.0 project and replace "ExternalInterpolator.h" with the one in this project  
* source /opt/neuromeka/NRMKFoundation/script/nrmk_env.sh  
* build project  
  * cmake -DROBOT_NAME=Indy7 -DSIMULATION=OFF  
  * make install  
* send the file to CB  
  * scp /home/junsu/Projects/indyframework2.0/deployment/* root@192.168.0.63:/home/user/release/TasksDeployment  
* Run TaskMan  
  * cd /home/user/release/TasksDeployment  
  * ./TaskManager -j indyDeploy.json  

# Launch RVIZ
* roslaunch "$TF_GMT_ETASL_DIR"/launch/gui_custom_robots_joint_panel.launch  

# Launch Panda simulator
* roslaunch panda_gazebo panda_world.launch start_moveit:=false   

# Launch franka ros interface  
* visualization: roslaunch franka_visualization franka_visualization.launch robot_ip:=192.168.0.13 load_gripper:=true  
* launch interface: roslaunch franka_interface interface.launch

# Launch Indy simulator (CadKit)
* run TaskManager (simulator mode)  on STEP
  * cd /home/user/release/IndyFramework3.0 && ./TaskManager -j indyDeploy.json
* open cadkit (NRMK Launcher -> CadKit
  * Insert Indy7 model ("C:\Program Files (x86)\neuromeka\NRMKFoundation\bin\models\Indy7\Model_Indy7.wrl")
  * Connect -> Enter ip of STEP PC -> Connect
 
# Recommended Tools  
* jupyter 
  * sudo apt install python3-notebook python-notebook jupyter jupyter-core python-ipykernel  
  * host setting  
* Teamviewer (autostart, password)  
* GitKraken  
* PyCharm, Clion  
  * add "export PATH=$PATH:{}/bin" to .bashrc  
* openssh-server  
  * sudo apt-get install openssh-server
  * sudo service ssh start
  
# How to make xacro for multi-robot  
* find xacro file in the description package for target robot  
* copy the xacro file to "$TF_GMT_ETASL_DIR"/robots  
* delete "world" link and joint connected to it  
* add macro:  
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
* change all item names: selectively replace {name="} with {"name="robotname${robot_id}}  
* include and call the xacro file in "custom_robots.urdf.xacro"  
* test generating URDF file  
  * rosrun xacro xacro "$TF_GMT_ETASL_DIR"robots/custom_robots.urdf.xacro \> "$TF_GMT_ETASL_DIR"robots/custom_robots.urdf  
* run rviz  
  * roslaunch "$TF_GMT_ETASL_DIR"/launch/gui_custom_robots_joint_panel.launch  
  * for "not unique" error, remove it from individual xacro files and include the item on the top of "custom_robots.urdf.xacro"  


# Check final .bashrc  

export PATH=$PATH:~/.local/bin  
  
\# cuda  
export PATH=$PATH:/usr/local/cuda-10.1/bin  
export CUDADIR=/usr/local/cuda-10.1  
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.1/lib64  
  
\# OROCOS  
source $HOME/orocos-install/orocos-2.9_ws/install_isolated/setup.bash  
  
\# ros  
alias eb='nano ~/.bashrc'  
alias sb='source ~/.bashrc'  
alias gs='git status'  
alias gp='git pull'  
alias cw='cd ~/catkin_ws'  
alias cs='cd ~/catkin_ws/src'  
alias cm='cd ~/catkin_ws && catkin_make'  
source /opt/ros/melodic/setup.bash  
source $HOME/catkin_ws/devel/setup.bash  
export ROS_MASTER_URI=http://localhost:11311  
export ROS_HOSTNAME=localhost  
  
\# etasl  
\#source $HOME/etasl_ws/devel/setup.sh  
source /home/junsu/etasl_ws/etasl/ws/etasl-py/devel/setup.bash  
export TESSERACT_SUPPORT_DIR='/home/junsu/Projects/tf_gmt/eTaSL/ws_ros/devel/share/tesseract_support'  

\# tf_gmt  
export TF_GMT_ETASL_DIR=/home/junsu/Projects/tf_gmt/eTaSL/  
source "$TF_GMT_ETASL_DIR"ws_ros/devel/setup.bash  
  
\# JetBrains  
export PATH=$PATH:/home/junsu/pycharm-2020.2/bin  
export PATH=$PATH:/home/junsu/clion-2020.2/bin  

