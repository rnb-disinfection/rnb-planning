# RNB-Planning v0.2.0 Release Notes (2021-11-18)

## Preparing release
### Branches to merge
* feature-manual-mobile
* feature-demo1108
* feature-perception-test?
* feature-sweep-data-new?
### Branches to remove
* feature-wiping-area-dataset
* feature-cpp
### Added features
#### Detector
* MultiICP
  - Implement as Detector subclass
    - Detection rule using mmdet & MultiICP 
  - Need to add usage script
* CombinedDetector
  - Camera class as singletone
#### Planning
* PDDLStream (Pick&Place)
  - Need to add usage script
* **BindingTransform**
  - ***pscene.sample_leaf_state()*** now returns *to_state* only. *redundancy_dict* is contained in ***to_state.binding_state***.
  - ***State.binding_state*** is a dictionary of **BindingTransform**. The redundancy can be accessed at ***BindingTransform.redundancy***.
  - To adjust redundancy, call ***BindingTransform.set_redundancy()*** and ***BindingTransform.update_transforms()***.
* Interpolated path generation by incremental IK
### Removed features
#### eTaSL planner

## New features
* **TBD**  
  
## Deprecated features
* **TBD**    
  
## Known issues
* **WebUI **
  * 양쪽 창에 같은 탭을 띄울 경우 마지막에 띄운 탭의 버튼만 동작함.
* **Setup**
  * moveit_interface_py 빌드 전 apt update && sudo apt-get dist-upgrade 필요 - repository 등 설정 꼬여있던 부분 문제일 수 있음, 확인 필요.
* **BUG**
  * GraspChecker - sphere에 대한 collision checking 이상
  
  
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
