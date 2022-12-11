# ROS Setup
* ROS Noetic  
  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && sudo apt install curl \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
  && sudo apt-get update \
  && sudo apt-get install ros-noetic-desktop-full -y \
  && source /opt/ros/noetic/setup.bash \ 
  && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
  && sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y \
  && sudo rosdep init \
  && rosdep update \
  && sudo apt-get install ros-noetic-catkin python3-catkin-tools python3-catkin-pkg -y \
  && sudo apt-get install ros-noetic-move-base \
  && pip3 install rospkg
  ```
  * **RESTART TERMINAL!**  
  * If key server fails:
  ```bash
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  ```
  
* Moveit  
  ```bash
  sudo apt-get install -y ros-noetic-moveit ros-noetic-industrial-core ros-noetic-moveit-visual-tools ros-noetic-joint-state-publisher-gui
  ```  
* Gazebo  
  ```bash
  sudo apt-get install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-joint-trajectory-controller  
  ```
* UR package  
  * link: https://github.com/ros-industrial/universal_robot  
  ```bash
  cd $HOME/catkin_ws/src \
  && git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git \
  && cd $HOME/catkin_ws \
  && rosdep update \
  && rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src \
  && catkin_make -DCMAKE_BUILD_TYPE=Release  
  ```
* Indy package
  ```bash
  cd ~/catkin_ws/src && git clone -b  release-2.3 https://github.com/neuromeka-robotics/indy-ros \
  && cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
  ```
  * (not used now) To update Indy to 3.0, Follow instruction on external/IndyFramework3.0/ReadMe.md
* Franka package  
  ```bash
  sudo apt install ros-noetic-libfranka ros-noetic-franka-ros \
  && cd ~/catkin_ws \
  && git clone https://github.com/justagist/franka_ros_interface src/franka_ros_interface \
  && catkin_make -DCMAKE_BUILD_TYPE=Release \
  && source devel/setup.bash
  ```
  * Copy/move the franka.sh file to the root of the catkin_ws
  ```bash
  cp ~/catkin_ws/src/franka_ros_interface/franka.sh ~/catkin_ws/
  ```
  * ~~Change the values in the copied file (described in the file).~~
* python compatibility  
  ```bash
  pip install rospkg  
  ```
