# ros2_ws — ROS 2 워크스페이스

**ROS 2 패키지는 전부 `src/` 아래에 둡니다.** 파트 폴더(`sensor/` `slam/` …)에는 ROS 가 아닌 코드·설정을 둡니다. 패키지 이름은 `fd_` 접두어로 통일합니다.

| 패키지 | 담당 |
| --- | --- |
| `fd_interfaces` | 전원 |
| `fd_sensor` | A |
| `fd_slam` | B |
| `fd_sim_bridge` | D |
| `fd_inference` | D·E |

```bash
cd ros2_ws && colcon build --symlink-install     # build/ install/ log/ 는 .gitignore 대상
```
