# Introduction
[zed2](https://www.stereolabs.com/docs/body-tracking) 공식 사이트에서 body tracking을 통해 관절 11, 17번의 3D Pose를 linear interpolation을 통해 얻은
z=0인 부분을 target point로 정하여, 구하는 과정을 담은 프로젝트입니다.

이 프로젝트는 ZED 2 카메라의 body tracking 기능을 활용하여 관절 11번과 17번의 3D 포즈 데이터를 수집하고, 
이를 선형 보간(linear interpolation) 방식으로 처리하여 z=0인 지점을 타겟 포인트로 설정하는 작업을 포함합니다. 
이 과정을 통해 얻어진 타겟 포인트는 특정 좌표로 이동하기 위한 Navigation의 입력으로 활용될 수 있으며, 본 프로젝트는 이러한 처리 과정을 ROS2로 구현되었습니다.

if ZED2 Camera가 없다면, 다른 3D HPE 모델을 사용해서 얻은 Pose로도 대체 가능합니다. (target_point.py에서 INDEX 코드 수정)
이 프로젝트는 ZED 2 카메라를 사용하여 실시간으로 인체의 3D 포즈를 추적하는 것을 기반으로 하지만, ZED 2 카메라가 없는 경우에도 다른 3D 인체 포즈 추정(Human Pose Estimation, HPE) 모델을 활용하여 유사한 데이터를 얻을 수 있습니다. 
사용자는 target_point.py 내의 INDEX 코드를 수정하여 다른 HPE 모델로부터 얻은 포즈 데이터를 프로젝트에 통합할 수 있습니다.

하지만, zed-ros2-interface 대신 3D HPE의 데이터를 담을 interface를 커스텀마이징 해야합니다.

## Installation
# Prerequisites
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble
- zed-ros2-interfaces
- ZED SDK
- CUDA dependency

# Build the package
The ros2_calculate_goalpoint is a colcon package.

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/7zjatl7/ros2_calculate_goalpoint.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
cd ..
sudo apt update
colcon build --symlink-install
```

```bash
ros2 launch get_target_point send_goal_point.launch.py
```







