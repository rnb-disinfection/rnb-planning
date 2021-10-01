# Build tensorflow from source code
* This document introduces how to build the tensorflow from source code.
* Building TF from source code is specifically required to use TensorRT>=7.2.3 with tensorflow<=2.6.0
* Until tensorflow 2.6.0, the compiled versions on pip is linked with TensorRT=7.2.2
* TensorRT>=7.2.3 is needed to it is the first version that supports Conv3D
* This document is based on the package versions below.
  * nvidia driver == 460.91.03
  * CUDA == 11.2 ***update 1***
  * cuDNN == 8.1.1
  * TensorRT == 7.2.3
  * Tensorflow == 2.6.0

## Prepare dependencies
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
sudo apt-get install --no-install-recommends nvidia-driver-460=460.91.03-0ubuntu1 \
&& sudo apt-get install cuda-drivers=460.91.03-1
```

* ***[IMPORTANT]*** Reboot!!!  
  * Check that GPUs are visible using the command: nvidia-smi

* Install cuda - this will install many dependancies like cudart,cublas of version 11.2
```bash
sudo apt-get install --no-install-recommends cuda-11-2=11.2.1-1
```

* Install cudnn
```bash
sudo apt-get install --no-install-recommends \
  libcudnn8=8.1.1.33-1+cuda11.2  \
  libcudnn8-dev=8.1.1.33-1+cuda11.2
```

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

* ***[IMPORTANT]*** Reboot!!!  
  * Check that GPUs are visible using the command: nvidia-smi

* TensorRT 7.2.3 (compatible with cudnn 8.1.1 above)
  * Download *TensorRT 7.2.3 for Linux and CUDA 11.2* from https://developer.nvidia.com/tensorrt.
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
    * apt-get update sometimes generates errors. You can ignore and continue mostly.
    ```bash  
    sudo apt-get -y install --no-install-recommends libnvinfer7=7.2.3-1+cuda11.1 \
      libnvinfer-dev=7.2.3-1+cuda11.1 \
      libnvinfer-plugin7=7.2.3-1+cuda11.1 \
      libnvinfer-plugin-dev=7.2.3-1+cuda11.1 \
      libnvparsers7=7.2.3-1+cuda11.1 \
      libnvparsers-dev=7.2.3-1+cuda11.1 \
      libnvonnxparsers7=7.2.3-1+cuda11.1 \
      libnvonnxparsers-dev=7.2.3-1+cuda11.1 \
      libnvinfer-samples=7.2.3-1+cuda11.1 \
      && sudo apt-get -y install tensorrt=7.2.3.4-1+cuda11.1 \
      && sudo apt-get -y install python-libnvinfer=7.2.3-1+cuda11.1 \
      && sudo apt-get -y install python-libnvinfer-dev=7.2.3-1+cuda11.1 \
      && sudo apt-get -y install python3-libnvinfer=7.2.3-1+cuda11.1 \
      && sudo apt-get -y install python3-libnvinfer-dev=7.2.3-1+cuda11.1 \
      && sudo apt-get install uff-converter-tf \
      && pip3 install keras2onnx
    ```
  * Add path in .bashrc
  ```bash
  echo 'export PATH=$PATH:/usr/src/tensorrt/bin' >> ~/.bashrc
  ```

* **[IMPORTANT]** Restart the terminal!
  
## Install Tensorflow by pip3 or building from source
### CASE 1: Install with pip3
```bash
pip3 install tensorflow-gpu==2.6.0
```
  * Tensorflow is installed. 

### CASE 2: Build from source code
* Install Bazelisk
```bash
sudo apt install apt-transport-https curl gnupg \
&& curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg \
&& sudo mv bazel.gpg /etc/apt/trusted.gpg.d/ \
&& echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list \
&& sudo apt update && sudo apt install bazel-3.7.2 \
&& sudo mv /usr/bin/bazel-3.7.2 /usr/bin/bazel
```
* get tensorflow
```bash
cd ~ && git clone https://github.com/tensorflow/tensorflow.git && cd tensorflow \
&& git checkout r2.6
```
* configure the system build
```bash
./configure
```
  * For configuration, use default values if not mentioned below. (just press enter)
  * **Python library path**: */usr/local/lib/python3.6/dist-packages*
  * **CUDA support**: *y*
  * **TensorRT support**: *y*
* Build tensorflow - adjust memory size and job number in the following command depending on your system - *OOM* error is not monitorable and very frequent when building tensorflow.
```bash
bazel build --config=opt  --local_ram_resources=16384 --jobs=10 //tensorflow/tools/pip_package:build_pip_package
```
* Build whl package
```bash
./bazel-bin/tensorflow/tools/pip_package/build_pip_package /tmp/tensorflow_pkg
```
* Install whl
```bash
pip3 install /tmp/tensorflow_pkg/tensorflow-2.6.0-cp36-cp36m-linux_x86_64.whl
```

## Supplementary
### nvidia, cuda, cudnn, tensorrt Clean Unsintall
* uninstall all related packages
```bash
sudo apt-get remove --purge '^nvidia-.*' \
&& sudo apt-get remove --purge 'cuda*' \
&& sudo apt-get remove --purge 'nv-tensorrt*' \
&& sudo apt-get remove --purge 'libnv*' \
&& sudo apt-get remove --purge 'libcudnn*' \
&& sudo apt-get remove --purge 'libcublas*' \
&& sudo rm -rf /usr/local/cuda
```
* check installed lists
```bash
apt list --installed | grep "nvidia-*"
apt list --installed | grep "cuda-*"
apt list --installed | grep "libcudnn-*"
apt list --installed | grep "libnv-*"
```
* manually remove all packages listed above
```bash
sudo apt-get remove --purge {package name}
```
* **[IMPORTANT]** Reboot the system
