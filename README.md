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
- **Pixracer → TELEM2 → uXRCE-DDS → ROS 2**: топики PX4 для ROS-части.
  Запуск Micro XRCE-DDS Agent на роботе пока не входит в `rover_bringup`.
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

| аргумент    | что делает                                               |
|-------------|----------------------------------------------------------|
| `cameras`   | камеры из `config/cameras.yaml`                          |
| `stream`    | RTSP `video_streamer` для QGC (порт 8554)                |
| `web_video` | `web_video_server` (MJPEG по HTTP, порт 8081)            |

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

### Управление с геймпада

1. В QGC подключите геймпад: `Vehicle Setup → Joystick`, откалибруйте, назначьте кнопки (arm/disarm, режимы), включите
   `Enable joystick`. QGC отправляет `MANUAL_CONTROL` по MAVLink через mavlink-router.
2. В PX4 установите параметр `COM_RC_IN_MODE` = «RC и джойстик» (`RC and Joystick`), чтобы
   принимались и MANUAL_CONTROL от QGC, и RC.
3. ELRS-приёмник остаётся резервным каналом: при пропадании джойстика управление возможно с пульта.
   Не забудьте настроить `COM_RC_LOSS_T` и failsafe в QGC.

### udev-правила

Контейнеры используют стабильные имена вместо `ttyACM*`/`video*`. Узнать атрибуты:
`udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'`.
Пример `/etc/udev/rules.d/99-rover.rules` (подставьте свои `idVendor`/`idProduct`/`serial`):

```
# Pixracer (PX4, USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="rover_fcu", MODE="0666"
# RPLidar (CP210x USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="rover_lidar", MODE="0666"
# вторая USB-камера: только узел видеопотока (index0), не metadata
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", ATTR{index}=="0", SYMLINK+="rover_cam2", MODE="0666"
```

Применить: `sudo udevadm control --reload && sudo udevadm trigger`.
