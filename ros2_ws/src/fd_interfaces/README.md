# fd_interfaces

- **담당**: 전원
- **내용**: 공용 메시지 · 서비스 정의, 토픽 이름 · frame_id 규약

아직 ROS 패키지가 아닙니다(`package.xml` 없음 → colcon 이 건너뜀). 이 폴더에서 생성하세요:

```bash
cd ros2_ws/src
ros2 pkg create fd_interfaces --build-type ament_python   # C++ 이면 ament_cmake
```
