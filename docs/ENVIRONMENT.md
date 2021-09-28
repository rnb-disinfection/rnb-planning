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

## Setup NVIDIA cuda 11.1 and cudnn 8.1 for tf 2.5.0 (below is official guide from homepage)

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
sudo apt-get install --no-install-recommends nvidia-driver-460
```
* ***[IMPORTANT]*** Reboot!!!  
  Check that GPUs are visible using the command: nvidia-smi


* Install development and runtime libraries (~4GB)
```bash
cd ~/NVIDIA_TMP \
&& wget https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/libnvinfer7_7.2.3-1+cuda11.1_amd64.deb \
&& sudo apt install ./libnvinfer7_7.2.3-1+cuda11.1_amd64.deb \
&& sudo apt-get update \
&& sudo apt-get install --no-install-recommends \
  cuda-11-2 \
  libcudnn8=8.1.1.33-1+cuda11.2  \
  libcudnn8-dev=8.1.1.33-1+cuda11.2
```

* ***[IMPORTANT]*** Reboot!!!  
  Check that GPUs are visible using the command: nvidia-smi

* Add PATH variables to environment
```bash
echo 'export PATH=$PATH:/usr/local/cuda-11.2/bin' >> ~/.bashrc \
&& echo 'export CUDADIR=/usr/local/cuda-11.2' >> ~/.bashrc \
&& echo 'if [ -z $LD_LIBRARY_PATH ]; then' >> ~/.bashrc \
&& echo '  export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64' >> ~/.bashrc \
&& echo 'else' >> ~/.bashrc \
&& echo '  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.2/lib64' >> ~/.bashrc \
&& echo 'fi' >> ~/.bashrc  
```

* ***[IMPORTANT]*** Restart terminal!!!

* Install TensorRT. Requires that libcudnn8 is installed above.
```bash
sudo apt-get install -y --no-install-recommends libnvinfer7=7.2.3-1+cuda11.1 \
  libnvinfer-dev=7.2.3-1+cuda11.1 \
  libnvinfer-plugin7=7.2.3-1+cuda11.1
```

  
* Install tensorflow
```bash
pip3 install tensorflow-gpu==2.5.0
```

* TensorRT 7.2.3 (compatible with cudnn 8.1.1 above)
  * Download *TensorRT 7.2.3 for Linux and CUDA 11.1* from https://developer.nvidia.com/tensorrt.
    * Select Ubundu 18.04 deb file
  * Follow the installation guide from https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-723/install-guide/index.html
    * Example:
      ```bash
      os="ubuntu1804" \
      && tag="cuda11.1-trt7.2.3.4-ga-20210226" \
      && sudo dpkg -i nv-tensorrt-repo-${os}-${tag}_1-1_amd64.deb \
      && sudo apt-key add /var/nv-tensorrt-repo-${os}-${tag}/7fa2af80.pub \
      && sudo apt-get update
      ```
      * apt-get update sometimes generates errors. Ignore and continue.
      ```bash
        && sudo apt-get -y install libnvinfer-dev=7.2.3-1+cuda11.1 libnvinfer-plugin-dev=7.2.3-1+cuda11.1 libnvparsers-dev=7.2.3-1+cuda11.1 libnvonnxparsers-dev=7.2.3-1+cuda11.1 libnvinfer-samples=7.2.3-1+cuda11.1 \
        && sudo apt-get -y install tensorrt \
        && sudo apt-get -y install python-libnvinfer-dev \
        && sudo apt-get -y install python3-libnvinfer-dev \
        && sudo apt-get install uff-converter-tf

      ```
  * Add path in .bashrc
  ```bash
  echo 'export PATH=$PATH:/usr/src/tensorrt/bin' >> ~/.bashrc
  ```
  * Install keras2onnx
  ```bash
  pip3 install keras2onnx
  ```

* ***[IMPORTANT]*** in TF 2.4.0, add below to python script before using tensorflow or you get "no algorithm worked" error! (no need to explicitly use session)
```python
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)
```
