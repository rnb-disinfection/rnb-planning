# Hardware setup
## Camera Setup
* Azure Kinect  
  * setup microsoft repository  
  ```console
  curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - \
  && sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod \
  && sudo apt-get update  
  ```
  * install sdk  
  ```console
  sudo apt install libk4a1.4 \
  && sudo apt install libk4a1.4-dev \
  && sudo apt install k4a-tools  
  ```
  * allow non-root usage  
    - Download [third-party/azure/99-k4a.rules](../third-party/azure/99-k4a.rules) in this project.  
    - From the downloaded directory,  
  ```console
  sudo cp ./99-k4a.rules /etc/udev/rules.d/  
  ```
  * Detach and reattach Azure Kinect devices if attached during this process.  
  * install open3d  
  ```console
  pip install open3d  
  ```
  * test kinect by executing "k4aviewer" on terminal  
  
* Realsense
  * Follow below process (UBUNTU 18.04 case, ref: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
  ```console
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  && sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u \
  && sudo apt-get install librealsense2-dkms \
  && sudo apt-get install librealsense2-utils \
  && sudo apt-get install librealsense2-dev \
  && sudo apt-get install librealsense2-dbg
  ```
  * Reconnect the Intel RealSense depth camera and verify the installation.
  ```console
  realsense-viewer
  ```
  * install pyrealsense2  
  ```console
  pip install pyrealsense2  
  ```
  
## Setup and launch panda repeater
* setup panda_ros_repeater on panda master pc (https://github.com/Cucumberkjs/panda_ros_repeater.git)  
* launch panda command repeater on matser  
  ```console
  roslaunch panda_ros_repeater joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=false
  ```

## Setup and launch indy online tracker
* Download and install NRMK IndyFramework and PlatformSDK (framework 2.3.1 -> Platform 3.0.5)  
* Clone IndyFramework2.0 project and copy "external/IndyFramework2.0/IndyController/*" and "external/IndyFramework2.0/IndyHRI/*" from this project to corresponding folders in framework source.  
* build project  
  ```console
  source /opt/neuromeka/NRMKFoundation/script/nrmk_env.sh  
  cmake -DROBOT_NAME=Indy7 -DSIMULATION=OFF  
  make install  
  ```
* send the file to CB  
  ```console
  scp $HOME/Projects/external/IndyFramework2.0/deployment/* root@192.168.0.63:/home/user/release/TasksDeployment  
  ```
* Run TaskMan on CB, through ssh
  ```console
  cd /home/user/release/TasksDeployment  
  ./TaskManager -j indyDeploy.json  
  ```
