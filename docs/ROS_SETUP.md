# ROS Setup
* ROS Melodic  
  ```bash
  mkdir ~/ROS_TMP && cd ~/ROS_TMP \
  && wget https://raw.githubusercontent.com/orocapangyo/meetup/master/190830/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh \
  && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
  && sudo apt-get update \
  && sudo apt-get install ros-melodic-desktop-full -y \
  && sudo apt-get install ros-melodic-rqt* -y \
  && sudo apt-get install python-rosdep -y \
  && sudo rosdep init \
  && rosdep update \
  && sudo apt-get install python-rosinstall -y \
  && sudo apt-get install ros-melodic-catkin python-catkin-tools -y \
  && sudo apt-get install ros-melodic-move-base
  ```
  * **RESTART TERMINAL!**  
  
* Moveit  
  ```bash
  sudo apt-get install -y ros-melodic-moveit ros-melodic-industrial-core ros-melodic-moveit-visual-tools ros-melodic-joint-state-publisher-gui
  ```  
* Gazebo  
  ```bash
  sudo apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-trajectory-controller  
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
  sudo apt install ros-melodic-libfranka ros-melodic-franka-ros \
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
