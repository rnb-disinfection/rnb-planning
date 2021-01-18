# 1. Environment Setup  

## 1.1 Tensorflow base environment  
* Follow instruction in [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)    
   
## 1.2 ROS Setup  
* The planning framework is based on ROS  
* Follow instruction in [docs/ROS_SETUP.md](docs/ROS_SETUP.md)  

## 1.3 eTaSL  
* To use eTaSL planner and online planning, install eTaSL as shown [docs/ETASL_SETUP.md](docs/ETASL_SETUP.md)  
 
## 1.4 other dependencies    
* Python Package Dependencies  
  ```
  pip install colorama==0.3.9 llvmlite==0.31.0 numba==0.47.0
  pip install autograd && pip install --user pymanopt==0.2.4
  pip install dash==1.17.0 visdcc dash_split_pane
  pip install matplotlib trimesh pathlib protobuf grpcio numpy-stl sklearn filterpy paramiko SharedArray  
  pip3 install dill matplotlib sklearn opencv-python SharedArray  
  ```
  
* Install OMPL (takes >30min)
    * Download ompl install bash file [here](third-party/ompl/install-ompl-ubuntu.sh)
    * From the downloaded directory, 
        ```
        chmod +x ./install-ompl-ubuntu.sh
        ./install-ompl-ubuntu.sh --python
        ```
  
## 1.5 hardware setup
* Setup camera and robot driver/sdk following instructions in [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) 
  
# 2 Setup project  
## 2.1 Get project setup path  
* Download and add path to ~/.bashrc    
  ```
  mkdir ~/Projects && cd ~/Projects \
  && git clone https://github.com/rnb-disinfection/rnb-planning.git \
  && export RNB_PLANNING_DIR=$HOME/Projects/rnb-planning/ \
  && echo 'export RNB_PLANNING_DIR=$HOME/Projects/rnb-planning/' >> ~/.bashrc
  ```
  
## 2.2 Build custom etasl
* get custom etasl project from github and recompile etasl  
    ```
    cd ~/etasl/ws \
    && mv ./etasl ./etasl_bak && mv ./etasl-py ./etasl-py_bak \
    && cp -r $RNB_PLANNING_DIR/third-party/etasl/etasl ./ \
    && cp -r $RNB_PLANNING_DIR/third-party/etasl/etasl-py ./
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
  
## 2.3 build subprojects
* Build moveit-python interpreter, copy it and clean Release folder  
    ```
    sudo apt-get remove ros-melodic-ompl \
    && cd "$RNB_PLANNING_DIR"lib/moveit_interface_py \
    && chmod +x ./build.sh \
    && ./build.sh \
    && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib \
    && echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
    ```

* build openGJK
    ```
    cd "$RNB_PLANNING_DIR"lib/openGJK/lib \
    && cmake -DCMAKE_BUILD_TYPE=Release \
    && make
    ```
  
* build custom workspace  
    ```
    cd "$RNB_PLANNING_DIR"ws_ros && rm -rf build devel && catkin_make -DCMAKE_BUILD_TYPE=Release  
    source "$RNB_PLANNING_DIR"ws_ros/devel/setup.bash
    echo 'source "$RNB_PLANNING_DIR"ws_ros/devel/setup.bash' >> ~/.bashrc
    ```

## 2.4 Check shell environemnt settings
* ~/.bashrc should contain following lines.  
   ```
   # export PATH=$PATH:~/.local/bin  

   # ORCOS
   source $HOME/orocos-install/orocos-2.9_ws/install_isolated/setup.bash

   # CUDA
   export PATH=$PATH:/usr/local/cuda-11.0/bin
   export CUDADIR=/usr/local/cuda-11.0
   if [ -z $LD_LIBRARY_PATH ]; then
     export LD_LIBRARY_PATH=/usr/local/cuda-11.0/lib64
   else
     export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.0/lib64
   fi

   # ROS
   alias eb='nano ~/.bashrc'
   alias sb='source ~/.bashrc'
   alias gs='git status'
   alias gp='git pull'
   alias cw='cd ~/catkin_ws'
   alias cs='cd ~/catkin_ws/src'
   alias cm='cd ~/catkin_ws && catkin_make'
   source /opt/ros/melodic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   export ROS_MASTER_URI=http://localhost:11311
   export ROS_HOSTNAME=localhost

   # eTaSL
   source $HOME/etasl/ws/etasl-py/devel/setup.bash

   # RNB-PLANNING
   export RNB_PLANNING_DIR=$HOME/Projects/rnb-planning/

   # custom-ompl
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

   # custom-workspace
   source "$RNB_PLANNING_DIR"ws_ros/devel/setup.bash

   # JetBrains  
   export PATH=$PATH:$HOME/pycharm-2020.2/bin  
   export PATH=$PATH:$HOME/clion-2020.2/bin  
   ```
