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
  * echo "source $OROCOS/install_isolated/setup.bash" >> ~/.bashrc  
  * source /home/junsu/etasl_ws/etasl/ws/etasl-py/devel/setup.bash  
* Install UR package  
  * link: https://github.com/ros-industrial/universal_robot  
  * cd $HOME/catkin_ws/src  
  * git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git  
  * cd $HOME/catkin_ws  
  * rosdep update  
  * rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src  
  * catkin_make  
  
# Python packages
* ros compatibility  
  * pip3 install rospkg  
* misc.  
  * pip3 install matplotlib trimesh  

# Setup project  
* (rebuild custom workspace - not used now, workspace dependecny problem)  
  * (cd eTaSL/ws_ros && rm -rf build devel && catkin make)  
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

# Final .bashrc  

export PATH=$PATH:~/.local/bin  
  
\# cuda  
export PATH=$PATH:/usr/local/cuda-10.1/bin  
export CUDADIR=/usr/local/cuda-10.1  
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.1/lib64  
  
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
  
\# OROCOS  
source $HOME/orocos-install/orocos-2.9_ws/install_isolated/setup.bash  
  
\# etasl  
\#source $HOME/etasl_ws/devel/setup.sh  
source /home/junsu/etasl_ws/etasl/ws/etasl-py/devel/setup.bash  
  
\# tf_gmt  
export TF_GMT_ETASL_DIR=/home/junsu/Projects/tf_gmt/eTaSL/  
\#source "$TF_GMT_ETASL_DIR"ws_ros/devel/setup.bash  
  
