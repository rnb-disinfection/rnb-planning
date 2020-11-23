# Basic Environment Setup (Tensorflow base)
* Turn off "Fast boot" & "Secure boot" on Bios
* Ubuntu 18.04  
* install gcc7 & gcc5
  ```
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-7 g++-7 gcc-7-multilib g++-7-multilib \
  && sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-5 g++-5 gcc-5-multilib g++-5-multilib  
  ```
* set gcc alternative versions:  
  ```
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20 \
  && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40 \
  && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20 \
  && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 40  
  ```
* check gcc/g++ alternative version  
  ```
  sudo update-alternatives --display gcc && sudo update-alternatives --display g++  
  ```
* pip
  ```
  sudo apt-get install python3-pip && pip3 install --upgrade pip \
  && sudo apt-get install python-pip && pip install --upgrade pip \
  && pip3 install setuptools==41.0.0 \
  && pip install setuptools==41.0.0  
  ```

* vga driver: follow tensorflow recommendation - https://www.tensorflow.org/install/gpu?hl=ko#install_cuda_with_apt
  ```
  # Add NVIDIA package repositories
  mkdir ~/NVIDIA_TMP && cd ~/NVIDIA_TMP \
  && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.1.243-1_amd64.deb \
  && sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub \
  && sudo dpkg -i cuda-repo-ubuntu1804_10.1.243-1_amd64.deb \
  && sudo apt-get update \
  && wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb \
  && sudo apt install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb \
  && sudo apt-get update

  # Install NVIDIA driver
  sudo apt-get install --no-install-recommends nvidia-driver-450
  ```  
  * **Reboot! Check that GPUs are visible using the command: nvidia-smi**
* reboot and continue with cuda & cudnnn  
  ```
  # Install development and runtime libraries (~4GB)
  sudo apt-get install --no-install-recommends \
      cuda-10-1 \
      libcudnn7=7.6.5.32-1+cuda10.1 \
      libcudnn7-dev=7.6.5.32-1+cuda10.1

  # Add PATH variables to environment
  echo 'export PATH=$PATH:/usr/local/cuda-10.2/bin' >> ~/.bashrc \
  && echo 'export CUDADIR=/usr/local/cuda-10.2' >> ~/.bashrc \
  && echo 'if [ -z $LD_LIBRARY_PATH ]; then' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64' >> ~/.bashrc \
  && echo 'else' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64' >> ~/.bashrc \
  && echo 'fi' >> ~/.bashrc  

  # Install TensorRT. Requires that libcudnn7 is installed above.
  sudo apt-get install -y --no-install-recommends libnvinfer6=6.0.1-1+cuda10.1 \
      libnvinfer-dev=6.0.1-1+cuda10.1 \
      libnvinfer-plugin6=6.0.1-1+cuda10.1
  ```
  * Install tensorflow
  ```
  pip3 install tensorflow
  ```
  * test GPU usage in python3
  ```
  import tensorflow as tf
  tf.test.is_gpu_available()
  from tensorflow.python.client import device_lib
  device_lib.list_local_devices()
  ```
    
# Python Package Dependencies
* numba  
  ```
  pip install colorama==0.3.9 llvmlite==0.31.0 numba==0.47.0  
  ```
* pymanopt
  ```
  pip install autograd && pip install --user pymanopt==0.2.4 
  ```
* Dash
  ```
  pip install dash==1.17.0 visdcc dash_split_pane
  ```
* misc.  
  ```
  pip install matplotlib trimesh pathlib protobuf grpcio numpy-stl sklearn filterpy paramiko  
  ```

# ROS Setup
* ROS Melodic  
  ```
  mkdir ~/ROS_TMP && cd ~/ROS_TMP \
  && wget https://raw.githubusercontent.com/orocapangyo/meetup/master/190830/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh \
  && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
  && sudo apt-get update && sudo apt-get upgrade -y \
  && sudo apt-get install ros-melodic-desktop-full -y \
  && sudo apt-get install ros-melodic-rqt* -y \
  && sudo apt-get install python-rosdep -y \
  && sudo rosdep init \
  && rosdep update \
  && sudo apt-get install python-rosinstall -y \
  && sudo apt-get install ros-melodic-catkin python-catkin-tools -y
  ```
  * **RESTART TERMINAL!**
* Moveit  
  ```
  sudo apt-get install -y ros-melodic-moveit ros-melodic-industrial-core ros-melodic-moveit-visual-tools ros-melodic-joint-state-publisher-gui  
  ```
* Gazebo  
  ```
  sudo apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-trajectory-controller  
  ```
* UR package  
  * link: https://github.com/ros-industrial/universal_robot  
  ```
  cd $HOME/catkin_ws/src \
  && git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git \
  && cd $HOME/catkin_ws \
  && rosdep update \
  && rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src \
  && catkin_make -DCMAKE_BUILD_TYPE=Release  
  ```
* Indy package
  ```
  cd ~/catkin_ws/src && git clone -b  release-2.3 https://github.com/neuromeka-robotics/indy-ros \
  && cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
  ```
  * (not used now) To update Indy to 3.0, Follow instruction on IndyFramework3.0/ReadMe.md
* Franka package  
  ```
  sudo apt install ros-melodic-libfranka ros-melodic-franka-ros \
  && cd ~/catkin_ws \
  && git clone https://github.com/justagist/franka_ros_interface src/franka_ros_interface \
  && catkin_make -DCMAKE_BUILD_TYPE=Release \
  && source devel/setup.bash
  ```
  * Copy/move the franka.sh file to the root of the catkin_ws
  ```
  cp ~/catkin_ws/src/franka_ros_interface/franka.sh ~/catkin_ws/
  ```
  * Change the values in the copied file (described in the file).
* python compatibility  
  ```
  pip install rospkg  
  ```
  
# eTaSL  
* Follow install process in the link - https://etasl.pages.gitlab.kuleuven.be/install-new.html  
  ```
  cd ~ \
  && git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/docker/etasl-install.git etasl \
  && cd etasl \
  && source install-dependencies.sh
  ```
* **ADD** "source $HOME/orocos-install/orocos-2.9_ws/install_isolated/setup.bash" on top of ~/.bashrc  
* switch gcc and g++ version to 7 before installing etasl
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
* install etasl base
  ```
  source $HOME/etasl/etasl-install.sh
  ```
* switch gcc and g++ version to 5 before installing etasl-py
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
* install etasl-py base
  ```
  source $HOME/etasl/ws/etasl/devel/setup.sh \
  && source python-etasl-install.sh \
  && source $HOME/etasl/ws/etasl-py/devel/setup.bash \
  && echo 'source $HOME/etasl/ws/etasl-py/devel/setup.bash' >> ~/.bashrc \
  ```
* switch gcc and g++ version back to 7 before installing etasl-py
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
* ***If eTaSL is slow***, re-compile packages in release mode (has to be under 300ms with 200 constraints 500 step)  

    
# Camera Setup
* Azure Kinect  
  * setup microsoft repository  
  ```
  curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - \
  && sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod \
  && sudo apt-get update  
  ```
  * install sdk  
  ```
  sudo apt install libk4a1.4 \
  && sudo apt install libk4a1.4-dev \
  && sudo apt install k4a-tools  
  ```
  * allow non-root usage  
    * Download 'azure/99-k4a.rules' in this project. From the downloaded directory,
  ```
  sudo cp ./99-k4a.rules /etc/udev/rules.d/  
  ```
    * Detach and reattach Azure Kinect devices if attached during this process.  
  * install open3d  
  ```
  pip install open3d  
  ```
  * test kinect by executing "k4aviewer" on terminal  
  
* Realsense
  * Follow instruction in https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md  
    * BELOW if for UBUNTU 18.04
  ```
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  && sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u \
  && sudo apt-get install librealsense2-dkms \
  && sudo apt-get install librealsense2-utils
  ```
  * install pyrealsense2  
  ```
  pip install pyrealsense2  
  ```
  
# Setup project  
* Build custom etasl
  * get custom etasl project from github and recompile etasl
  ```
  cd ~/etasl/ws \
  && mv ./etasl ./etasl_bak && mv ./etasl-py ./etasl-py_bak \
  && git clone https://github.com/Cucumberkjs/etasl.git \
  && git clone https://github.com/Cucumberkjs/etasl-py.git
  ```
  * **[IMPORTANT]** comment out "source $HOME/etasl/ws/etasl-py/devel/setup.bash" in ~/.bashrc
  * restart terminal  
  * switch gcc and g++ version to 7 before installing etasl
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
  * rebuild etasl 
  ```
  cd ~/etasl/ws/etasl \
  && sudo rm -rf devel && sudo rm -rf build && catkin_make -DCMAKE_BUILD_TYPE=Release \
  && source $HOME/etasl/ws/etasl/devel/setup.bash   
  ```
  * switch gcc and g++ version to 5 before installing etasl-py
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
  * rebuild etasl-py 
  ```
  cd ~/etasl/ws/etasl-py \
  && sudo rm -rf devel && sudo rm -rf build && catkin_make -DCMAKE_BUILD_TYPE=Release \
  && source $HOME/etasl/ws/etasl-py/devel/setup.bash   
  ```
  * **[IMPORTANT]** uncomment "source $HOME/etasl/ws/etasl-py/devel/setup.bash" in ~/.bashrc
  * restart terminal  
  * switch gcc and g++ version back to 7
  ```
  sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
  ```
  
* Get project and add path to ~/.bahsrc
  ```
  mkdir ~/Projects && cd ~/Projects \
  && git clone https://github.com/Cucumberkjs/tamp_etasl.git \
  && export TAMP_ETASL_DIR=$HOME/Projects/tamp_etasl/eTaSL/ \
  && echo 'export TAMP_ETASL_DIR=$HOME/Projects/tamp_etasl/eTaSL/' >> ~/.bashrc
  ```
  * build openGJK
  ```
  cd "$TAMP_ETASL_DIR"openGJK/lib \
  && cmake -DCMAKE_BUILD_TYPE=Release \
  && make
  ```
  * build custom workspace  
  ```
  cd "$TAMP_ETASL_DIR"ws_ros && rm -rf build devel && catkin_make -DCMAKE_BUILD_TYPE=Release  
  source "$TAMP_ETASL_DIR"ws_ros/devel/setup.bash
  echo 'source "$TAMP_ETASL_DIR"ws_ros/devel/setup.bash' >> ~/.bashrc
  ```
  * start roscore if it's not active  
  ```
  nohup roscore &  
  ```

# Setup and launch panda repeater
* setup panda_ros_repeater on panda master pc (https://github.com/Cucumberkjs/panda_ros_repeater.git)  
* launch panda command repeater on matser  
  ```
  roslaunch panda_ros_repeater joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=false
  ```

# Setup and launch indy online tracker
* Download and install NRMK IndyFramework and PlatformSDK (framework 2.3.1 -> Platform 3.0.5)  
* Clone IndyFramework2.0 project and copy "IndyFramework2.0/IndyController/*" and "IndyFramework2.0/IndyHRI/*" frp, this project to corresponding folders in framework source.  
* build project  
  ```
  source /opt/neuromeka/NRMKFoundation/script/nrmk_env.sh  
  cmake -DROBOT_NAME=Indy7 -DSIMULATION=OFF  
  make install  
  ```
* send the file to CB  
  ```
  scp $HOME/Projects/indyframework2.0/deployment/* root@192.168.0.63:/home/user/release/TasksDeployment  
  ```
* Run TaskMan on CB, through ssh
  ```
  cd /home/user/release/TasksDeployment  
  ./TaskManager -j indyDeploy.json  
  ```

# TIPS 
* Launching RVIZ
  ```
  roslaunch "$TAMP_ETASL_DIR"launch/gui_custom_robots_joint_panel.launch 
  ``` 

* Launch franka ros interface  
  * visualization:
  ```
  roslaunch franka_visualization franka_visualization.launch robot_ip:=192.168.0.13 load_gripper:=true  
  ```
  * launch interface: 
  ```
  roslaunch franka_interface interface.launch
  ```
 
# Recommended Tools  
* jupyter 
  ```
  sudo apt install python3-notebook python-notebook jupyter jupyter-core python-ipykernel  
  ```
  * do server setting  
* Teamviewer (autostart, password)  
* GitKraken  
* PyCharm, Clion  
  * add "export PATH=$PATH:{}/bin" to .bashrc  
* openssh-server  
  ```
  sudo apt-get install openssh-server -y && sudo service ssh start
  ```

# Check final .bashrc  
```
export PATH=$PATH:~/.local/bin  
  
\# cuda  
export PATH=$PATH:/usr/local/cuda-10.2/bin
export CUDADIR=/usr/local/cuda-10.2
if [ -z $LD_LIBRARY_PATH ]; then
  export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64
else
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64
  
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
source $HOME/etasl/ws/etasl-py/devel/setup.bash
# export TESSERACT_SUPPORT_DIR='$HOME/Projects/tamp_etasl/eTaSL/ws_ros/devel/share/tesseract_support'  

\# tamp_etasl  
export TAMP_ETASL_DIR=$HOME/Projects/tamp_etasl/eTaSL/
source "$TAMP_ETASL_DIR"ws_ros/devel/setup.bash
  
\# JetBrains  
export PATH=$PATH:$HOME/pycharm-2020.2/bin  
export PATH=$PATH:$HOME/clion-2020.2/bin  
```
