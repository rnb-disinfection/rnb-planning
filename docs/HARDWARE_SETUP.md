# Hardware setup
## Camera Setup
* Azure Kinect  
  * setup microsoft repository  
  ```bash
  curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - \
  && sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod \
  && sudo apt-get update  
  ```
  * install sdk  
  ```bash
  sudo apt install libk4a1.4 \
  && sudo apt install libk4a1.4-dev \
  && sudo apt install k4a-tools  
  ```
  * allow non-root usage  
    - Download [third-party/azure/99-k4a.rules](../third-party/azure/99-k4a.rules) in this project.  
    - From the downloaded directory,  
  ```bash
  sudo cp ./99-k4a.rules /etc/udev/rules.d/  
  ```
  * Detach and reattach Azure Kinect devices if attached during this process.  
  * install open3d  
  ```bash
  pip install open3d  
  ```
  * test kinect by executing "k4aviewer" on terminal  
  
* Realsense
  * Follow below process (UBUNTU 18.04 case, ref: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
  ```bash
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  && sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u \
  && sudo apt-get install librealsense2-dkms \
  && sudo apt-get install librealsense2-utils \
  && sudo apt-get install librealsense2-dev \
  && sudo apt-get install librealsense2-dbg
  ```
  * Reconnect the Intel RealSense depth camera and verify the installation.
  ```bash
  realsense-viewer
  ```
  * install pyrealsense2  
  ```bash
  pip install pyrealsense2  
  ```
  
## Setup and launch indy and panda
* setup [rnb-control](https://github.com/rnb-disinfection/rnb-control) on indy and panda
* launch panda command repeater on matser  
  ```bash
  roslaunch panda_control joint_control_rnb.launch robot_ip:=192.168.0.13 load_gripper:=false
  ```

## (Expert) Adding a new hardware to the framework
* You need a xacro model file in src/robots to use a robot model in this framework.
    * To make xacro file compatible with this framework, read **How to make xacro for multi-robot** section in [docs/SUPPLEMENT_README.md](../docs/SUPPLEMENT_README.md)
* You also need to set planning parameters for your robot in [src/robots/kinematics.yaml](../src/robots/kinematics.yaml]) and [src/robots/ompl_planning.yaml](../src/robots/ompl_planning.yaml)
    * Check the contents of the file, mostly you can just copy the contents and change the name of the robot to get ready.
