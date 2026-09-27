# fd_sim_bridge

- **담당**: D
- **내용**: Isaac Sim ↔ ROS 2 토픽 연결 · 교사 데이터 기록 노드

아직 ROS 패키지가 아닙니다(`package.xml` 없음 → colcon 이 건너뜀). 이 폴더에서 생성하세요:

```bash
cd ros2_ws/src
ros2 pkg create fd_sim_bridge --build-type ament_python   # C++ 이면 ament_cmake
```
