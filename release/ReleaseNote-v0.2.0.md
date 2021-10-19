# RNB-Planning v0.1.0 Release Notes (2021-3-20)
## New features
* **TBD**  
  
## Deprecated features
* **None (First release)**  
  
  
## Dropped features
* **None (First release)**  
  
  
## Preparing features
* **4-A.3 LatticedChecker**  
  - pick, place
    - single process
    - *multi process is not supported with LatticedChecker, which uses GPU*
    
* **6. Web UI**
  - save/load buttons
  - auto page refresh
  
* **7.2 Using Tool to sweep**  
  - script
* **7.3 Multi Object Handing**  
  - script
  
  
## Known issues
* **WebUI **
  * 양쪽 창에 같은 탭을 띄울 경우 마지막에 띄운 탭의 버튼만 동작함.
* **Setup**
  * moveit_interface_py 빌드 전 apt update && sudo apt-get dist-upgrade 필요 - repository 등 설정 꼬여있던 부분 문제일 수 있음, 확인 필요.
  
  
## Requirements

### 1. Robot controller version
  - [rnb-control v0.2.0](https://github.com/rnb-disinfection/rnb-control/releases/tag/v0.2.0-panda)

### 2. Model
* **ReachChecker**
  - model/reach_svm/indy7.json
  - model/reach_svm/panda.json
  - 백업: 개인 이동식 하드디스크, 강준수
* **LatticedChecker**
  - model/latticized/indy7/20210222-134724
  - model/latticized/panda/20210222-145610
  - 백업: 개인 이동식 하드디스크, 강준수
