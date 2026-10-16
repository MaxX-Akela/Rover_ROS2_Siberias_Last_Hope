# Rover_ROS2_Siberias_Last_Hope
ROS2 image Rover: "Siberia’s Last Hope"

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy)
[![PX4](https://img.shields.io/badge/PX4-1.17-blue)](https://px4.io)
[![Docker](https://img.shields.io/badge/Docker-TINA-2496ED?logo=docker)](docker/)
![С++](https://img.shields.io/badge/С++-TINA-blue?logo=cpp)
![Python](https://img.shields.io/badge/python3-btw-blue?logo=python)
![Raspberry Pi 5](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?logo=raspberrypi&logoColor=white)

## PixHawk 2.4.8/PixRacer r15 [firmware](https://github.com/Pulse-Width-Modulation-Technologies/Rover-Autopilot)

## Связь, видео и управление

### Схема сети

```
                         ┌──────────────── ровер ─────────────────┐
                         │                                        │
 Pixracer ──USB──────────┼─► mavlink-router ──TCP 5760 / UDP 14550┼──┐
    │                    │   (docker, не ROS)                     │  │
    └──TELEM2─ uXRCE-DDS─┼─► ROS 2 ◄── камеры (camera_ros/usb_cam) │  │  Keenetic
                         │            │                           │  ├── Wi-Fi ──► ноутбук оператора
                         │            ├─► video_streamer ─RTSP 8554┼──┤            QGroundControl
                         │            └─► web_video_server  :8081 ─┼──┘
                         └─ Raspberry Pi 5, Ethernet 192.168.1.10 ─┘
```

- **Pixracer → USB → mavlink-router → QGC**: телеметрия, параметры, миссии, джойстик.
- **Pixracer → TELEM2 → uXRCE-DDS → ROS 2**: топики PX4 (`/fmu/...`), `/odom`, `/cmd_vel` → PX4.
  Micro XRCE-DDS Agent запускается в `bringup.launch.py` (`agent:=true`, `/dev/rover_telem2`, 921600).
- **Камеры → ROS-топики → `video_streamer` → QGC**: H.264 по RTSP. Камеру напрямую открывают только
  драйверы (`camera_ros`, `usb_cam`), стример читает кадры из топиков.

Адрес ровера и порты — в [`rover_bringup/config/network.yaml`](rover_bringup/config/network.yaml)
(`rover_ip`, `rtsp_port`, `web_video_port`). Сам адрес назначается на Pi (см. ниже), конфиг нужен,
чтобы стример и launch выводили правильные URL.

### Запуск

```sh
docker compose -f docker/compose.yaml --profile robot up --build
```

Профиль `robot` поднимает два сервиса: `ros` (`rover_bringup bringup.launch.py`) и `mavlink-router`.
Аргументы launch (по умолчанию все `true`):

```sh
ros2 launch rover_bringup bringup.launch.py cameras:=true stream:=true web_video:=true
```

| аргумент       | что делает                                                       |
|----------------|------------------------------------------------------------------|
| `cameras`      | камеры из `config/cameras.yaml`                                  |
| `stream`       | RTSP `video_streamer` для QGC (порт 8554)                        |
| `web_video`    | `web_video_server` (MJPEG по HTTP, порт 8081)                    |
| `agent`        | Micro XRCE-DDS Agent (`agent_dev`, `agent_baud`)                 |
| `fcu_bridge`   | `rover_fcu_bridge`: `/cmd_vel` → PX4, `/odom`                    |
| `teleop`       | `rover_teleop`: джойстик QGC → `/cmd_vel`                        |
| `control_mode` | `direct` или `velocity` (пусто = из `config/control.yaml`)       |

Конфиги `rover_bringup/config` примонтированы в контейнер `ros`, после правки достаточно перезапустить сервис.

### Статический IP на Pi

Роутер Keenetic, Pi — по Ethernet, `192.168.1.10`. Пример netplan (`/etc/netplan/50-rover.yaml`):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses: [192.168.1.10/24]
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [192.168.1.1]
```

Либо закрепите адрес за MAC-адресом Pi в Keenetic (Домашняя сеть → Список устройств → «Постоянный IP-адрес»).

### Камеры

`rover_bringup/config/cameras.yaml` — список камер (`name`, `driver: camera_ros|usb_cam`, `device`, `width`,
`height`, `fps`, `namespace`). По умолчанию одна камера `front` (camera_ros, 1280x720@30);
пример второй USB-камеры (`usb_cam`, `/dev/rover_cam2`, 640x480@30, `mjpeg2rgb`) закомментирован.
Камера публикует `/<namespace>/camera/image_raw`, например `/front/camera/image_raw`.

### QGroundControl (Windows)

Ноутбук должен быть в Wi-Fi роутера Keenetic.

**MAVLink** — `Application Settings → Comm Links → Add`:
- TCP: Server Address `192.168.1.10`, Port `5760`; или
- UDP: Server Address `192.168.1.10`, Port `14550`.

**Видео** — `Application Settings → Video`: Video Source = `RTSP Video Stream`,
RTSP URL = `rtsp://192.168.1.10:8554/front` (для второй камеры — `/rear` и т.д., по её `namespace`).

**Web-видео (запасной вариант, браузер/VLC)**: `http://192.168.1.10:8081/` — список топиков.
Прямой поток камеры `front`:
`http://192.168.1.10:8081/stream?topic=/front/camera/image_raw`
(варианты: `&type=mjpeg`, `&width=640&height=480&quality=50`).

### Управление: QGC → /cmd_vel → PX4

QGC остаётся интерфейсом оператора (видео, телеметрия, геймпад), но команды движения идут через ROS 2 в `/cmd_vel`.

```
геймпад ─► QGC ──MANUAL_CONTROL──► mavlink-router ─┬─✗─► Pixracer (MANUAL_CONTROL заблокирован)
                                                   └───► rover_teleop ──/cmd_vel──► rover_fcu_bridge ──uXRCE-DDS──► PX4 (Offboard)
                                                                                         ▲                            │
                                      ELRS (RC) ───────────────────── аварийный перехват ┘      /odom, TF odom→base_link ◄─┘
```

- **rover_teleop** (Python, `pymavlink`) принимает `MANUAL_CONTROL` с UDP-эндпоинта mavlink-router (`127.0.0.1:14560`)
  и публикует `geometry_msgs/Twist` в `/cmd_vel`. Параметры в `rover_bringup/config/control.yaml`: `max_linear`,
  `max_angular`, `deadzone`, `invert_linear`, `invert_angular`, `linear_axis`/`angular_axis`, `timeout`.
  Нет `MANUAL_CONTROL` дольше `timeout` (0.5 с) — публикуется нулевой Twist. Стик вперёд = `linear.x > 0`,
  стик руля вправо = `angular.z < 0` (REP 103).
- **mavlink-router** (`docker/mavlink-router/main.conf`): эндпоинту полётника запрещён `MANUAL_CONTROL`
  (`BlockMsgIdOut = 69`), поэтому PX4 не исполняет джойстик напрямую. Эндпоинту `teleop` разрешены только
  `HEARTBEAT` и `MANUAL_CONTROL`. rover_teleop шлёт heartbeat (sysid 1, компонент 191), иначе роутер не
  пересылает ему сообщения, адресованные системе 1.
- **rover_fcu_bridge** подписан на `/cmd_vel` и держит heartbeat `OffboardControlMode` 20 Гц. Нет `/cmd_vel`
  дольше `cmd_vel_timeout` (0.5 с) — ровер останавливается. Режим задаёт параметр `control_mode`
  (в `control.yaml` или `bringup.launch.py control_mode:=velocity`):

| `control_mode` | что уходит в PX4 | требования |
|---|---|---|
| `direct` (по умолчанию) | `/fmu/in/rover_throttle_setpoint` + `/fmu/in/rover_steering_setpoint`, `OffboardControlMode.thrust_and_torque` | ничего: ни одометрии, ни локальной позиции |
| `velocity` | `/fmu/in/trajectory_setpoint` (скорость), `OffboardControlMode.velocity` | валидная локальная позиция; заднего хода нет |

**Почему `direct` — это `rover_throttle/steering_setpoint`.** В `dds_topics.yaml` PX4 v1.17 для прямого управления
доступны три пути: `manual_control_input`, `actuator_motors/servos` и `rover_throttle/steering_setpoint`.
1. `manual_control_input` отфильтровывается `ManualControlSelector` при `COM_RC_IN_MODE` = «только RC»
   (`isRc(data_source)`), а это значение нужно, чтобы ELRS был аварийным перехватом. Не подходит.
2. `actuator_motors` обходит модуль ровера целиком (ограничения ускорения, обратную кинематику).
3. `rover_throttle_setpoint` (−1…1 вперёд) и `rover_steering_setpoint` (−1…1 вправо) с флагом
   `thrust_and_torque` — штатный «Throttle + Steering» из документации PX4 по offboard для ровера.
   `/cmd_vel` переводится так: `throttle = linear.x / max_linear`, `steering = −angular.z / max_angular`
   (обрезается до ±1). Если ровер крутится не в ту сторону — `invert_steering: true`.

В режиме `velocity` модуль `rover_differential` в PX4 v1.17 превращает вектор скорости в «скорость = |v|» и
«курс = направление v». Поэтому угловая скорость реализуется как курс, опережающий текущий на
`−angular.z · velocity_yaw_horizon` (0.5 с), а при нулевой команде отправляется скорость 0.02 м/с вдоль текущего курса
(нулевой вектор означал бы курс «на север»).

**`/odom`** (`nav_msgs/Odometry`, кадры `odom` → `base_link`) и TF `odom → base_link` строятся из
`/fmu/out/vehicle_odometry` с переводом NED/FRD → ENU/FLU (`rover_fcu_bridge/conversions.py`, покрыто тестами).
Скорость в `twist` — в системе `base_link`.

#### Настройка PX4 и порядок работы

1. `COM_RC_IN_MODE` = **0 (только RC)**: джойстик QGC в PX4 не принимается, ELRS — аварийный перехват
   (переключатель режима на пульте → Manual/Hold отключает Offboard).
2. Если ровер ездит без пульта в эфире: `COM_RCL_EXCEPT` — включить бит Offboard (значение 4),
   иначе PX4 уйдёт в failsafe по потере RC. Проверьте `COM_OF_LOSS_T` и `COM_OBL_RC_ACT`.
3. В QGC: `Vehicle Setup → Joystick` — откалибровать и включить `Enable joystick`. Если руль/газ
   на других осях, задайте `linear_axis`/`angular_axis`. Для оси `z` (газ 0…1000) используйте `z_center: 500`.
4. Взвести ровер и выбрать **Offboard** в меню режимов QGC (мост уже шлёт heartbeat). Либо одной командой:
   `ros2 service call /rover_fcu_bridge/arm_disarm rover_msgs/srv/ArmDisarm "{arm: true}"`.

#### Как жюри проверяет `/cmd_vel` и `/odom`

Внутри контейнера `ros` (`docker compose -f docker/compose.yaml --profile robot exec ros bash`) или на машине
с тем же `ROS_DOMAIN_ID`:

```sh
# одометрия и TF
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link

# ехать вперёд 0.3 м/с и поворачивать влево 0.2 рад/с (команду нужно повторять: watchdog 0.5 с)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"

# остановка: Ctrl+C (через 0.5 с мост сам остановит ровер) или явный ноль
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# что уходит в PX4
ros2 topic echo /fmu/in/rover_throttle_setpoint
ros2 topic echo /fmu/in/rover_steering_setpoint
```

Ровер должен быть взведён и в режиме Offboard (п. 4 выше). Джойстик из QGC и `ros2 topic pub` пишут в один и тот же
`/cmd_vel`, поэтому не запускайте их одновременно.

### udev-правила

Контейнеры используют стабильные имена вместо `ttyACM*`/`video*`. Узнать атрибуты:
`udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'`.
Пример `/etc/udev/rules.d/99-rover.rules` (подставьте свои `idVendor`/`idProduct`/`serial`):

```
# Pixracer (PX4, USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="rover_fcu", MODE="0666"
# RPLidar (CP210x USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="rover_lidar", MODE="0666"
# Pixracer TELEM2 через USB-UART для Micro XRCE-DDS Agent (если не используется GPIO UART Pi)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="TELEM2", SYMLINK+="rover_telem2", MODE="0666"
# вторая USB-камера: только узел видеопотока (index0), не metadata
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", ATTR{index}=="0", SYMLINK+="rover_cam2", MODE="0666"
```

Применить: `sudo udevadm control --reload && sudo udevadm trigger`.
