# Yolo Test
**Author** : <a href="https://github.com/mkdir-sweetiepie"><img src="https://img.shields.io/badge/Ji Hyeon Hong-white?style=flat&logo=github&logoColor=red"/></a>    
**Maintainor** : <hong091788@naver.com>   
**Bug / feature tracker** : https://github.com/mkdir-sweetiepie/yolo_test/issues                 
**Source** : https://github.com/mkdir-sweetiepie/yolo_test (branch : master)

## Description
You can check if the object detection using YOLO is working well 

## Build & Usage
### build from source code
```bash
cd ~/${workspace_name}_ws/src
git clone https://github.com/mkdir-sweetiepie/yolo_test.git
cd ..
catkin_make
```

### How to use 
```
roslaunch realsense2_camera rs_camera.launch 
rosrun yolo_master yolo_master
rosrun yolo_ui yolo_ui
```
