# Need cuda11.0, cudnn8.0 to use TensorRT for Conv3D
# Corresponding TF version is 2.4.0, but if installed as following, 
# "No algorithm worked!" error occurs.
* set repository  
  ```
  # Add NVIDIA package repositories
  mkdir ~/NVIDIA_TMP && cd ~/NVIDIA_TMP \
  && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin \
  && sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
  && wget https://developer.download.nvidia.com/compute/cuda/11.0.3/local_installers/cuda-repo-ubuntu1804-11-0-local_11.0.3-450.51.06-1_amd64.deb \
  && sudo dpkg -i cuda-repo-ubuntu1804-11-0-local_11.0.3-450.51.06-1_amd64.deb \
  && sudo apt-key add /var/cuda-repo-ubuntu1804-11-0-local/7fa2af80.pub \
  && sudo apt-get update

  # Install NVIDIA driver
  sudo apt-get install --no-install-recommends nvidia-driver-460
  ```  
  * **Reboot! Check that GPUs are visible using the command: nvidia-smi**
* reboot and continue with cuda & cudnnn  
  ```
  # Install development and runtime libraries (~4GB)
  sudo apt-get install -y --no-install-recommends \
      cuda=11.0.3-1 \
      libcudnn8=8.0.5.39-1+cuda11.0 \
      libcudnn8-dev=8.0.5.39-1+cuda11.0

  # Add PATH variables to environment
  echo 'export PATH=$PATH:/usr/local/cuda-11.0/bin' >> ~/.bashrc \
  && echo 'export CUDADIR=/usr/local/cuda-11.0' >> ~/.bashrc \
  && echo 'if [ -z $LD_LIBRARY_PATH ]; then' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=/usr/local/cuda-11.0/lib64' >> ~/.bashrc \
  && echo 'else' >> ~/.bashrc \
  && echo '  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.0/lib64' >> ~/.bashrc \
  && echo 'fi' >> ~/.bashrc  

  # Install TensorRT. Requires that libcudnn8 is installed above.
  sudo apt-get install -y --no-install-recommends libnvinfer7=7.2.2-1+cuda11.0 \
    libnvinfer-dev=7.2.2-1+cuda11.0 \
    libnvinfer-plugin7=7.2.2-1+cuda11.0
  ```
  * Install tensorflow
  ```
  pip3 install tensorflow-gpu==2.4.0
  ```