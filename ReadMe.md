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
  * catkin_make  
* Indy package
  * cd ~/catkin_ws/src && git clone -b  release-2.3 https://github.com/neuromeka-robotics/indy-ros
  * cd ~/catkin_ws && catkin_make
* Franka package  
  * sudo apt install ros-melodic-libfranka ros-melodic-franka-ros  
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
  * source /home/junsu/etasl_ws/etasl/ws/etasl-py/devel/setup.bash  


# Python packages
* ros compatibility  
  * pip3 install rospkg  
* misc.  
  * pip3 install matplotlib trimesh  

# Setup project  
* rebuild custom workspace  
  * cd eTaSL/ws_ros && rm -rf build devel && catkin make  
* add below to ~./bashrc
  * export TF_GMT_ETASL_DIR=/home/junu/Projects/tf_gmt/eTaSL/  
  * source "$TF_GMT_ETASL_DIR"ws_ros/devel/setup.bash  

# Launch RVIZ
* roslaunch "$TF_GMT_ETASL_DIR"/launch/gui_ur10.launch  

# Recommended Tools  
* jupyter 
  * sudo apt install python3-notebook python-notebook jupyter jupyter-core python-ipykernel  
  * host setting  
* Teamviewer (autostart, password)  
* GitKraken  
* PyCharm  

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
* include xacro call it in "custom_robots.urdf.xacro"  
* run rosrun xacro xacro custom_robots.urdf.xacro \> custom_robots.urdf  

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
  
\# tf_gmt  
export TF_GMT_ETASL_DIR=/home/junsu/Projects/tf_gmt/eTaSL/  
source "$TF_GMT_ETASL_DIR"ws_ros/devel/setup.bash  