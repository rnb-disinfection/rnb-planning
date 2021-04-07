# Basic Environment Setup (Tensorflow base)
* Turn off "Fast boot" & "Secure boot" on Bios
* Ubuntu 18.04  
* install gcc7 & gcc5
```bash
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-7 g++-7 gcc-7-multilib g++-7-multilib \
  && sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-5 g++-5 gcc-5-multilib g++-5-multilib  
  ```
* set gcc alternative versions:  
```bash
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20 \
  && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40 \
  && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20 \
  && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 40  
  ```
* check gcc/g++ alternative version  
```bash
  sudo update-alternatives --display gcc && sudo update-alternatives --display g++  
  ```
* pip
```bash
  sudo apt-get install python3-pip && pip3 install --upgrade pip \
  && sudo apt-get install python-pip && pip install --upgrade pip \
  && pip3 install setuptools==41.0.0 \
  && pip install setuptools==41.0.0  
  ```

## Setup NVIDIA cuda 11.0 and cudnn 8.0 for tf 2.4.0 (below is official guide from homepage)
* Below is for RTX2080 and nvidia graphic driver version 450.
* Check driver version compatibility with your graphic card and follow the latest official guide (https://www.tensorflow.org/install/gpu)
* Update repositories
```bash
  # Add NVIDIA package repositories
  mkdir ~/NVIDIA_TMP && cd ~/NVIDIA_TMP \
  && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin \
  && sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
  && sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub \
  && sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /" \
  && sudo apt-get update \
  && wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb \
  && sudo apt install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb \
  && sudo apt-get update
  ```

* Install NVIDIA driver
```bash
  sudo apt-get install --no-install-recommends nvidia-driver-450
  ```
* ***[IMPORTANT]*** Reboot!!!  
  Check that GPUs are visible using the command: nvidia-smi


* Install development and runtime libraries (~4GB)
```bash
  cd ~/NVIDIA_TMP \
  && wget https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/libnvinfer7_7.1.3-1+cuda11.0_amd64.deb \
  && sudo apt install ./libnvinfer7_7.1.3-1+cuda11.0_amd64.deb \
  && sudo apt-get update \
  && sudo apt-get install --no-install-recommends \
    cuda-11-0 \
    libcudnn8=8.0.4.30-1+cuda11.0  \
    libcudnn8-dev=8.0.4.30-1+cuda11.0
  ```

* Add PATH variables to environment
```bash
  echo 'export PATH=$PATH:/usr/local/cuda-11.0/bin' >> ~/.bashrc \
  && echo 'export CUDADIR=/usr/local/cuda-11.0' >> ~/.bashrc \
  && echo 'if [ -z $LD_LIBRARY_PATH ]; then' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=/usr/local/cuda-11.0/lib64' >> ~/.bashrc \
  && echo 'else' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.0/lib64' >> ~/.bashrc \
  && echo 'fi' >> ~/.bashrc  
  ```

* ***[IMPORTANT]*** Restart terminal!!!

* Install TensorRT. Requires that libcudnn8 is installed above.
```bash
  sudo apt-get install -y --no-install-recommends libnvinfer7=7.1.3-1+cuda11.0 \
    libnvinfer-dev=7.1.3-1+cuda11.0 \
    libnvinfer-plugin7=7.1.3-1+cuda11.0
  ```

  
* Install tensorflow
```bash
  pip3 install tensorflow-gpu==2.4.0
  ```

* ***[IMPORTANT]*** in TF 2.4.0, add below to python script before using tensorflow or you get "no algorithm worked" error! (no need to explicitly use session)
  ```python
  from tensorflow.compat.v1 import ConfigProto
  from tensorflow.compat.v1 import InteractiveSession

  config = ConfigProto()
  config.gpu_options.allow_growth = True
  session = InteractiveSession(config=config)
  ```
