# fd_inference

- **담당**: D·E
- **내용**: 추론 노드 (rclpy + torch) — E 는 모델·forward pass, D 는 노드·연동

아직 ROS 패키지가 아닙니다(`package.xml` 없음 → colcon 이 건너뜀). 이 폴더에서 생성하세요:

```bash
cd ros2_ws/src
ros2 pkg create fd_inference --build-type ament_python   # C++ 이면 ament_cmake
```
