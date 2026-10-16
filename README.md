# Rover_ROS2_Siberias_Last_Hope
ROS2 image Rover: "Siberia’s Last Hope"

## Docker

Всё лежит в `docker/`, контекст сборки — корень репозитория. `docker/COLCON_IGNORE` не даёт `colcon` заходить в эту папку.

| Образ | Dockerfile | Что внутри |
|---|---|---|
| `rover-ros` | `docker/ros/Dockerfile` | `ros:jazzy-ros-base`, зависимости через `rosdep`, MicroXRCEAgent v2.4.3, `third_party/rover.repos` (px4_msgs), сборка `colcon` в `/opt/rover` |
| `rover-px4-sitl` | `docker/px4/Dockerfile` (target `px4-sitl`) | PX4 SITL из `github.com/MaxX-Akela/PX4-Autopilot`, `PX4_GZ_STANDALONE=1`, airframe 50000 `gz_rover_differential` |
| `rover-gz` | `docker/px4/Dockerfile` (target `gz`) | Gazebo Harmonic (headless-сервер) с моделями и плагинами из PX4 |

Профили `docker/compose.yaml`:

```bash
# робот: ros2 launch rover_bringup bringup.launch.py
docker compose -f docker/compose.yaml --profile robot up --build

# симуляция: gz + px4-sitl + rover-ros (MicroXRCEAgent udp4 -p 8888 и rover_fcu_bridge)
docker compose -f docker/compose.yaml --profile sim up --build
```

Параметры (переменные окружения или `.env`): `PX4_REPO`, `PX4_VERSION` (по умолчанию `main`), `ROS_DOMAIN_ID`, `TAG`.
`PX4_VERSION` должен соответствовать версии `px4_msgs` в `third_party/rover.repos` (`release/1.16`).

Сборка без Docker:

```bash
vcs import --shallow src < third_party/rover.repos   # либо в third_party/
colcon build --symlink-install
```
