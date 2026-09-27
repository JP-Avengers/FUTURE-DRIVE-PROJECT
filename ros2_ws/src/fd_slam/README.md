# fd_slam

- **담당**: B
- **내용**: RTAB-Map launch · 파라미터 로드

아직 ROS 패키지가 아닙니다(`package.xml` 없음 → colcon 이 건너뜀). 이 폴더에서 생성하세요:

```bash
cd ros2_ws/src
ros2 pkg create fd_slam --build-type ament_python   # C++ 이면 ament_cmake
```
