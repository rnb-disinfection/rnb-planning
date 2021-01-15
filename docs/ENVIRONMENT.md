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
  sudo apt-get install -y --no-install-recommends \
      cuda-10-1 \
      libcudnn7=7.6.5.32-1+cuda10.1 \
      libcudnn7-dev=7.6.5.32-1+cuda10.1

  # Add PATH variables to environment
  echo 'export PATH=$PATH:/usr/local/cuda-10.1/bin' >> ~/.bashrc \
  && echo 'export CUDADIR=/usr/local/cuda-10.1' >> ~/.bashrc \
  && echo 'if [ -z $LD_LIBRARY_PATH ]; then' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=/usr/local/cuda-10.1/lib64' >> ~/.bashrc \
  && echo 'else' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.1/lib64' >> ~/.bashrc \
  && echo 'fi' >> ~/.bashrc  
  && echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64' >> ~/.bashrc \

  # Install TensorRT. Requires that libcudnn7 is installed above.
  sudo apt-get install -y --no-install-recommends libnvinfer6=6.0.1-1+cuda10.1 \
    libnvinfer-dev=6.0.1-1+cuda10.1 \
    libnvinfer-plugin6=6.0.1-1+cuda10.1
  ```
  * Install tensorflow
  ```
  pip3 install tensorflow-gpu==2.3.1
  ```
  * test GPU usage in python3
  ```
  import tensorflow as tf
  tf.test.is_gpu_available()
  from tensorflow.python.client import device_lib
  device_lib.list_local_devices()
  ```