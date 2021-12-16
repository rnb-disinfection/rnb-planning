# MMDET SETUP
* Overall procedure link: https://github.com/open-mmlab/mmdetection/blob/master/docs/get_started.md
* Install Pytorch runtime CUDA version  
  * For Cuda 11.0
    * link: https://pytorch.org/get-started/previous-versions/
    * To check the installation, use 'torch.cuda.is_available()' command.
  ```bash
  pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
  ```
* Install MMDetection  
  * Install mmcv-full corresponding cuda version
  ```bash
  pip3 install timm
  pip3 install mmcv-full -f https://download.openmmlab.com/mmcv/dist/cu110/torch1.7.1/index.html
  ```  
  * Install MMDetection  
  ```bash
  git clone https://github.com/open-mmlab/mmdetection.git \
  && cd mmdetection \
  && pip3 install -r requirements/build.txt \
  && pip3 install -v -e .  # or "python3 setup.py develop"  
  ```
* [Optional] Install apex  
  ```bash
  git clone https://github.com/NVIDIA/apex \
  && cd apex \
  && pip3 install -v --disable-pip-version-check --no-cache-dir --global-option="--cpp_ext" --global-option="--cuda_ext" ./  
  ```
# Use Swin-Transformer Object Detection
* For object detection & segmentation, we use cascade mask rcnn based swin-transformer
* Get the git repository
  ```bash
  cd ~
  git clone https://github.com/SwinTransformer/Swin-Transformer-Object-Detection.git
  export SWIN_TFOD_DIR=$HOME/Swin-Transformer-Object-Detection
  echo 'export SWIN_TFOD_DIR=$HOME/Swin-Transformer-Object-Detection' >> ~/.bashrc
   ```
* To verify whether MMDetection is installed correctly to use swin-transformer structure, we can run the following sample code to initialize a detector and inference a demo image.
  - [src/scripts/testing/Test_mmdet_installation.ipynb](../src/scripts/testing/Test_mmdet_installation.ipynb)
  
# Prepare Model & Config for SharedDetector and MultiICP
* create model folder for mmdet
```bash
mkdir $RNB_PLANNING_DIR/model/mmdet
```
* Download the pretrained model to model/mmdet/.  It wil be used from MultiICP Detector.
```bash
cd $RNB_PLANNING_DIR/model/mmdet \
&& wget https://github.com/SwinTransformer/storage/releases/download/v1.0.2/cascade_mask_rcnn_swin_base_patch4_window7.pth
```
* Clone the 'configs' folder in Swin-Transformer-Object-Detection to model/mmdet
  1) move to Swin-Transformer-Object-Detection directory
  2) copy config folder
  ```bash
  cp -rf ./configs $RNB_PLANNING_DIR/model/mmdet/
  ```
* Follow instruction in [release/3.1.MultiICP.ipynb](../release/3.1.MultiICP.ipynb) for detailed usage.
