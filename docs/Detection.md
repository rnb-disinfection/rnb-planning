# MMDET Setup
* Overall procedure link: https://github.com/open-mmlab/mmdetection/blob/master/docs/get_started.md
* Install Pytorch runtime CUDA version  
  * For Cuda 11.0
    * link: https://pytorch.org/get-started/previous-versions/
  ```bash
  pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
  ```
* Install MMDetection  
  * Install mmcv-full corresponding cuda version
  ```bash
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
* Verification
  To verify whether MMDetection is installed correctly, we can run the following sample code to initialize a detector and inference a demo image.
  ```python
  from mmdet.apis import init_detector, inference_detector

  config_file = 'configs/faster_rcnn/faster_rcnn_r50_fpn_1x_coco.py'
  # download the checkpoint from model zoo and put it in `checkpoints/`
  # url: https://download.openmmlab.com/mmdetection/v2.0/faster_rcnn/faster_rcnn_r50_fpn_1x_coco/faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth
  checkpoint_file = 'checkpoints/faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth'
  device = 'cuda:0'
  # init a detector
  model = init_detector(config_file, checkpoint_file, device=device)
  # inference the demo image
  inference_detector(model, 'demo/demo.jpg')  
  ```

