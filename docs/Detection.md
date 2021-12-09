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
  git clone https://github.com/SwinTransformer/Swin-Transformer-Object-Detection.git
   ```
* To verify whether MMDetection is installed correctly to use swin-transformer structure, we can run the following sample code to initialize a detector and inference a demo image.
  ```python
  import cv2
  from mmdet.apis import init_detector, inference_detector, show_result_pyplot

  config_file = 'configs/faster_rcnn/faster_rcnn_r50_fpn_1x_coco.py'
  # download the checkpoint from model zoo and put it in `checkpoints/`
  # url: https://download.openmmlab.com/mmdetection/v2.0/faster_rcnn/faster_rcnn_r50_fpn_1x_coco/faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth
  checkpoint_file = 'faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth'
  device = 'cuda:0'
  # init a detector
  model = init_detector(config_file, checkpoint_file, device=device)
  # load the demo image
  image = cv2.imraed('demo/demo.jpg', flags=cv2.IMREAD_UNCHANGED)
  # inference the demo image
  result = inference_detector(model, 'demo/demo.jpg')
  # show inference result
  show_result_pyplot(model, image, result)
  ```
* Download pretrained model of cascade mask rcnn based swin-transformer. It wil be used from MultiICP Detector
  * link: https://github.com/SwinTransformer/storage/releases/download/v1.0.2/cascade_mask_rcnn_swin_base_patch4_window7.pth

