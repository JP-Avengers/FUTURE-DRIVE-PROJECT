# ros2_ws — ROS 2 워크스페이스

**ROS 2 패키지는 전부 `src/` 아래에 둡니다.** 파트 폴더(`sensor/` `slam/` …)에는 ROS 가 아닌 코드·설정을 둡니다.

패키지는 필요할 때 각자 만듭니다. 이름은 다른 ROS 패키지와 겹치지 않게 `fd_` 접두어를 권장합니다.

```bash
cd ros2_ws/src
ros2 pkg create fd_<이름> --build-type ament_python   # C++ 이면 ament_cmake
cd .. && colcon build --symlink-install               # build/ install/ log/ 는 .gitignore 대상
```

패키지를 만들면 [`.github/CODEOWNERS`](../.github/CODEOWNERS)에 담당을 추가하세요.
