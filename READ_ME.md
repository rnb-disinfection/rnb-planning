# Requirements  
* Ubuntu 18.04
* Nvidia driver 440
  * sudo add-apt-repository ppa:graphics-drivers/ppa
  * sudo apt update
  * sudo apt-get install nvidia-driver-440
  * sudo reboot
* cuda 10.1 (add path)
* cudnn 7.6
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
  * source /opt/ros/melodic/setup.bash
*eTaSL


# Additional
* jupyter (host setting)
* Teamviewer (autostart, password)
* GitKraken
* PyCharm
