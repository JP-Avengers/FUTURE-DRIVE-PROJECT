# fd_sensor

- **담당**: A
- **내용**: D435i 녹화 launch · 정적 TF

아직 ROS 패키지가 아닙니다(`package.xml` 없음 → colcon 이 건너뜀). 이 폴더에서 생성하세요:

```bash
cd ros2_ws/src
ros2 pkg create fd_sensor --build-type ament_python   # C++ 이면 ament_cmake
```
