# Gazebo_MoveIt_TM12
<img width="204" height="270" alt="스크린샷 2026-01-23 15-44-56" src="https://github.com/user-attachments/assets/3157074b-e30a-4971-9310-5ce44e94ecf2" />
<img width="156" height="330" alt="스크린샷 2026-01-22 13-35-11" src="https://github.com/user-attachments/assets/b067c676-3fd4-45fd-9158-9cb4a6c527ea" />
<img width="1914" height="891" alt="스크린샷 2026-02-10 13-51-47" src="https://github.com/user-attachments/assets/e70eb0e9-08cf-4f24-b5f9-0dcdde3d0685" />

tm12 (with surface gripper) pick and place sequence

# Environment

- Ubuntu 22.04

- ROS2 Humble

- MoveIt2

# To Modify

- Fix joint sequence (to move at once)

- Add Surface Gripper Collision

- Add Surface Gripper (To Grasp Objects)

- Jump to other positions (each joint)

- Damping, Stiffness of Joint 1~6 (in macro files)

# Origin

tm12 urdf xacro / tm12 launch py + not dae

# Conveyor in GBTP

기존에 ratio 1.0이었을 때 - manipulator가 object에 접근하지 않았을 때 rtf는 30, 접근했을 때 rtf 17-20

ratio 0.0001로 decimate했을 때 - manipulator가 object에 접근하지 않았을 때 rtf 70-75, 접근했을 때 47-50

---------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------

libignition-gazebo-track-controller-system.so--------------> 이 둘은

libignition-gazebo-tracked-vehicle-system.so---------------> 궤도 차량용 플러그인

---------------------------------------------------------------------------------------------------------

libignition-gazebo-joint-controller-system.so--------------> 이 둘은

libignition-gazebo-joint-position-controller-system.so-----> 조인트 관련 플러그인

---------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------

joint position 선형으로 나오지 않을 시, 런치 파일에서 ros gz bridge /clock 체크할 것

++ dynamic gripper plugin 추후에 추가




