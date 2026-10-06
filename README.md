# ROS 2 Based Mobile Robot Platform

An autonomous, differential-drive mobile robot platform featuring real-time motor control, 2D SLAM mapping, Nav2 autonomous navigation, and a responsive mobile web teleoperation interface. Built entirely with open-source software and low-cost hardware.

---

## Academic Context & Attribution

- **Institution:** Yıldız Technical University (YTU), Department of Computer Engineering
- **Project:** Computer Engineering Capstone / Senior Design Project
- **Developer:** Abdülsamet Çetinkaya (ID: 22011099)
- **Advisor:** Asst. Prof. Dr. Erkan Uslu
- **Date:** June 2026

---

## Key Features

- **Distributed Master-Slave Architecture:** Time-critical, real-time control is separated from high-level perception and planning to eliminate CPU bottlenecks.
  - **Slave (ESP32-S3):** 50 Hz discrete PID velocity controller with feedforward gain, hardware encoder interrupts (3200 CPR), and Pololu Dual VNH5019 motor driver control.
  - **Master (Raspberry Pi 4, 2 GB):** Executes the full ROS 2 stack onboard (micro-ROS agent, odometry estimation, SLAM Toolbox, Nav2 navigation stack, and WebSocket bridge). Eliminates network latency, clock drift, and TF synchronization issues.
- **Micro-ROS Serial Bridge:** High-throughput serial communication between ESP32-S3 and Raspberry Pi 4 running ROS 2 Jazzy.
- **Embedded SBC Performance Optimization:** Specifically tuned parameters (AMCL particle bounds, DWB trajectory samples, costmap rates) enable stable operation on a memory-constrained 2 GB Raspberry Pi with end-to-end latency below 35 ms.
- **Dual Operational Modes:**
  - **2D SLAM Mapping & Teleoperation:** Real-time environment mapping with `slam_toolbox` (online async) and remote map persistence via `nav2_map_server`'s `map_saver_server`.
  - **Autonomous Navigation:** Global path planning (NavFn) and obstacle avoidance (DWB Local Planner) with automatic AMCL global localization.
- **Command Multiplexing & Safety Override:** `twist_mux` prioritizes teleoperation input (`/cmd_vel_joy`) over autonomous controller commands (`/cmd_vel_nav`) for immediate manual takeover.
- **Responsive Mobile Web Dashboard:** Zero-install HTML5 Canvas interface with dual virtual joysticks (`nipplejs`), live map & scan rendering, drag-to-set navigation goals, and an emergency stop trigger.

---

## System Architecture

```mermaid
flowchart TD
    subgraph UI ["User Interface Layer (Smartphone / Tablet)"]
        WebUI["Mobile Web Dashboard (index.html)<br/>- HTML5 Canvas Visualization<br/>- Dual Virtual Joysticks (nipplejs)<br/>- 2D Goal Pose Placement<br/>- Remote Map Saving & E-Stop"]
    end

    subgraph Pi ["Master Layer (Raspberry Pi 4 - 2GB RAM / ROS 2 Jazzy)"]
        RosBridge["rosbridge_server & rosapi<br/>(WebSocket Port 9090)"]
        TwistMux["twist_mux<br/>Priority Arbiter"]
        OdomNode["odometry_node<br/>- Dead Reckoning<br/>- Broadcasts /odom & TF (odom -> base_link)"]
        TrajPub["trajectory_publisher<br/>Publishes /odom_path & /slam_path"]
        RPLidar["rplidar_ros & static_tf<br/>Publishes /scan & TF (base_link -> laser)"]
        
        subgraph NavStack ["Autonomous & Mapping Stack"]
            SLAM["slam_toolbox (Online Async)"]
            MapServer["nav2_map_server & map_saver_server"]
            AMCL["nav2_amcl (Particle Filter)"]
            Nav2["nav2_minimal<br/>(Controller, Planner, Behaviors, BT Navigator)"]
        end
        
        MicroROSAgent["micro_ros_agent<br/>(Serial Transport @ 115200 baud)"]
    end

    subgraph ESP ["Slave Layer (ESP32-S3 Microcontroller)"]
        MicroROSClient["Micro-ROS Client Node<br/>(/esp32_hardware_node)"]
        PIDController["50 Hz Discrete PID Velocity Controller<br/>Feedforward + Anti-Windup Clamping"]
        MotorDriver["Pololu Dual VNH5019 H-Bridge Driver"]
        Encoders["Optical Quadrature Encoders<br/>(3200 CPR, Hardware ISR)"]
    end

    %% Communications
    WebUI <-->|WebSocket JSON / CBOR| RosBridge
    RosBridge -->|/cmd_vel_joy| TwistMux
    RosBridge -->|/goal_pose| Nav2
    RosBridge <-->|Call /map_saver_server/save_map| MapServer
    
    Nav2 -->|/cmd_vel_nav| TwistMux
    TwistMux -->|/cmd_vel| MicroROSAgent
    
    RPLidar -->|/scan| SLAM
    RPLidar -->|/scan| AMCL
    RPLidar -->|/scan| Nav2
    
    SLAM -->|/map & TF (map -> odom)| TrajPub
    AMCL -->|TF (map -> odom)| TrajPub
    OdomNode -->|/odom & TF (odom -> base_link)| TrajPub
    OdomNode -->|/odom| Nav2
    
    MicroROSAgent <-->|UART Serial (/dev/ttyACM0)| MicroROSClient
    MicroROSClient -->|/left_motor_vel, /right_motor_vel| OdomNode
    MicroROSClient --> PIDController
    PIDController --> MotorDriver
    Encoders -->|Hardware Interrupts| PIDController
```

---

## Hardware Specifications & Bill of Materials

| Component | Model / Specification | Function |
| :--- | :--- | :--- |
| **High-Level Computer (Master)** | Raspberry Pi 4 Model B (2 GB RAM) | ROS 2 Jazzy, SLAM, Nav2, rosbridge |
| **Low-Level Microcontroller (Slave)** | ESP32-S3 DevKitC-1 (Xtensa Dual-Core) | Real-time 50 Hz PID motor loop, encoder ISRs |
| **Motor Driver** | Pololu Dual VNH5019 Motor Driver Shield | Dual H-Bridge, up to 12A continuous per channel |
| **Actuators** | 2x DC Gearmotors with Quadrature Encoders | Differential drive, 3200 counts per wheel revolution |
| **2D Laser Scanner** | RPLiDAR A1M8 (360°, 12m range, 5.5–10 Hz) | 2D environment perception for SLAM and AMCL |
| **Chassis & Wheels** | 2-Wheel Differential + Caster, Wheel Diam: 70 mm, Track: 300 mm | Mobile robot mechanical structure |
| **Power Distribution** | 3S LiPo Battery (11.1V – 12.6V) + Dual LM2596 Buck Converters | Isolated 5V 3A rails for Raspberry Pi and ESP32 |

---

## Pinout & Wiring Configuration

### ESP32-S3 to Pololu VNH5019 & Encoders

| Signal | ESP32-S3 GPIO | Driver / Sensor Pin | Description |
| :--- | :--- | :--- | :--- |
| **Motor Left IN1A** | GPIO 17 | Pololu M1INA | Left motor direction A |
| **Motor Left IN1B** | GPIO 16 | Pololu M1INB | Left motor direction B |
| **Motor Left PWM** | GPIO 15 | Pololu M1PWM | Left motor speed control (PWM) |
| **Motor Right IN2A** | GPIO 6 | Pololu M2INA | Right motor direction A |
| **Motor Right IN2B** | GPIO 5 | Pololu M2INB | Right motor direction B |
| **Motor Right PWM** | GPIO 4 | Pololu M2PWM | Right motor speed control (PWM) |
| **Encoder Left Phase A** | GPIO 10 | Encoder Left A | Hardware interrupt (CHANGE) |
| **Encoder Left Phase B** | GPIO 11 | Encoder Left B | Direction sensing |
| **Encoder Right Phase A** | GPIO 9 | Encoder Right A | Hardware interrupt (CHANGE) |
| **Encoder Right Phase B** | GPIO 18 | Encoder Right B | Direction sensing |

### Inter-Device Connections

- **Raspberry Pi 4 to ESP32-S3:** USB Serial (`/dev/ttyACM0`, 115200 baud).
- **Raspberry Pi 4 to RPLiDAR A1:** USB Serial (`/dev/rplidar` or `/dev/ttyUSB0`, 115200 baud).

---

## Repository Structure

```
ROS2_based_mobile_platform_project/
├── README.md                           # Comprehensive project documentation
├── mobil_arayuz/                       # Mobile Web Interface
│   └── index.html                      # HTML5 Canvas + NippleJS + ROSLIB dashboard
├── mobile_interface/ -> mobil_arayuz   # Convenience symlink
├── project_codes_pi_2/                 # Main ROS 2 package (Raspberry Pi 4)
│   ├── CMakeLists.txt                  # Build script for ROS 2 package
│   ├── package.xml                     # Package metadata and dependencies
│   ├── EU_ARA_22011099.pdf             # Academic project technical report
│   ├── config/                         # Configuration and parameter files
│   │   ├── nav2_params_pi.yaml         # Resource-tuned Nav2 & AMCL parameters
│   │   └── twist_mux_params.yaml       # Priority multiplexer configuration
│   ├── launch/                         # ROS 2 launch files
│   │   ├── robot_launch.py             # Hardware bringup (ESP32 bridge + odom + LiDAR)
│   │   ├── mapping_launch.py           # SLAM Toolbox online mapping bringup
│   │   ├── otonom_launch.py            # Complete autonomous navigation bringup
│   │   ├── nav2_minimal_pi_launch.py   # Minimal Nav2 lifecycle nodes
│   │   └── rosbridge_launch.py         # WebSocket bridge for Web UI
│   └── src/                            # ROS 2 C++ source code
│       ├── odometry_node.cpp           # Dead-reckoning odometry & TF broadcaster
│       ├── pi_kinematics_node.cpp      # Host-side differential drive kinematics
│       └── trajectory_publisher.cpp    # Path accumulation & drift diagnostics
└── esp32_test_2/                       # ESP32-S3 Micro-ROS firmware
    ├── platformio.ini                  # PlatformIO build & library configuration
    └── src/
        └── main.cpp                    # 50 Hz PID motor control & Micro-ROS client
```

---

## Installation & Setup

### 1. ESP32-S3 Firmware Setup

1. Install [PlatformIO](https://platformio.org/) (via VS Code or CLI).
2. Connect the ESP32-S3 to your development machine via USB.
3. Navigate to the firmware directory and flash:
   ```bash
   cd esp32_test_2
   pio run --target upload
   ```
4. Verify serial monitor output:
   ```bash
   pio device monitor -b 115200
   ```

### 2. Raspberry Pi 4 (ROS 2 Jazzy) Setup

1. Ensure Ubuntu 24.04 LTS (or compatible) with ROS 2 Jazzy is installed on the Raspberry Pi.
2. Install prerequisite packages:
   ```bash
   sudo apt update
   sudo apt install -y \
       ros-jazzy-navigation2 \
       ros-jazzy-nav2-bringup \
       ros-jazzy-slam-toolbox \
       ros-jazzy-twist-mux \
       ros-jazzy-rplidar-ros \
       ros-jazzy-rosbridge-server \
       ros-jazzy-micro-ros-agent
   ```
3. Set up the USB serial udev rules for the RPLiDAR:
   ```bash
   echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/rplidar.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   sudo usermod -a -G dialout $USER
   ```
4. Clone or copy `project_codes_pi_2` into your ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cp -r /path/to/project_codes_pi_2 ~/ros2_ws/src/
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select project_codes_pi_2
   source ~/ros2_ws/install/setup.bash
   ```

---

## Operating Instructions

### Mode 1: 2D SLAM Mapping & Teleoperation

Use this mode to drive the robot around a new environment and generate an occupancy grid map.

1. Launch the SLAM mapping bringup on the Raspberry Pi:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch project_codes_pi_2 mapping_launch.py serial_port:=/dev/ttyACM0
   ```
2. Open the mobile dashboard in a browser on your phone/tablet connected to the same Wi-Fi network:
   - File path: `mobil_arayuz/index.html` (can be served via `python3 -m http.server 8000`).
   - Enter the robot's IP address: `ws://<ROBOT_IP>:9090` and tap **Connect**.
3. Drive the robot smoothly through the space using the on-screen joysticks.
4. When mapping is complete, tap the **Save Map** button in the dashboard, enter a name (e.g., `lab_floor`), and confirm. The map files (`lab_floor.yaml` and `lab_floor.pgm`) will be saved under `~/maps/`.

### Mode 2: Autonomous Navigation

Use this mode to navigate autonomously on a previously saved map.

1. Launch the autonomous navigation bringup:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch project_codes_pi_2 otonom_launch.py \
       map:=/home/asametpi/maps/lab_floor.yaml \
       serial_port:=/dev/ttyACM0
   ```
2. Open the mobile dashboard and connect to `ws://<ROBOT_IP>:9090`.
3. The system will automatically trigger global localization after 12 seconds (`/reinitialize_global_localization`). Drive the robot a few meters manually to let AMCL converge.
4. **Set a Navigation Goal:** Touch and drag anywhere on the map in the dashboard. The red arrow indicates the target position and heading. Release to dispatch the goal.
5. The robot will compute a global path (bright green) and navigate autonomously while avoiding dynamic obstacles.
6. **Safety Override:** To intervene at any point, touch either joystick to immediately override autonomous navigation, or tap the large red **STOP** button to execute an emergency halt.

---

## Raspberry Pi 4 (2 GB) Performance Optimization

Running a modern ROS 2 autonomous navigation stack on an embedded single-board computer with 2 GB RAM poses significant resource constraints. The following specific optimizations were designed and validated to prevent CPU throttling, memory exhaustion, and control loop timeouts:

| Parameter | Default (PC) | Tuned (Pi 4 2GB) | Rationale |
| :--- | :--- | :--- | :--- |
| **AMCL Max Particles** | 5000 | **1500** | Drastically reduces particle filter computational load while retaining reliable convergence in indoor spaces. |
| **AMCL Min Particles** | 500 | **200** | Minimizes CPU usage once localized. |
| **AMCL Laser Beams (`max_beams`)** | 60 | **40** | Subsamples LiDAR scan rays for likelihood field matching, lowering raycast CPU usage. |
| **Controller Frequency** | 10–20 Hz | **5.0 Hz** | Prevents missed deadline warnings on the DWB local controller; sufficient for speeds up to 0.35 m/s. |
| **DWB Velocity Samples (`vx_samples`)** | 20 | **6** | Reduces trajectory evaluation search space from >400 combinations to 36 candidates per cycle. |
| **Local Costmap Update Rate** | 5.0 Hz | **3.0 Hz** | Reduces memory bandwidth and obstacle layer raytrace computation. |
| **Global Costmap Update Rate** | 1.0–2.0 Hz | **1.0 Hz (pub: 0.5 Hz)** | Static global map requires infrequent updates. |
| **Auxiliary Nav2 Nodes** | Active | **Disabled** | Omitted collision monitors, dockers, and costmap smoothers to save ~450 MB RAM. |

**Benchmark Results:**
- **Control Loop Latency:** Average 32.4 ms (target: < 50 ms).
- **RAM Usage:** ~1.3 GB out of 2.0 GB during active navigation (zero swapping).
- **CPU Load:** 65–80% across 4 cores without thermal throttling.

---

## ROS 2 Interface Reference

### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Dead-reckoning odometry computed from wheel encoder pulses. |
| `/odom_path` | `nav_msgs/msg/Path` | Accumulated raw odometry trajectory (shows drift over time). |
| `/slam_path` | `nav_msgs/msg/Path` | Corrected robot path in the `map` coordinate frame. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Final multiplexed motor velocity command sent to ESP32. |
| `/left_motor_vel` | `std_msgs/msg/Int32` | Left wheel encoder velocity feedback [counts/s]. |
| `/right_motor_vel` | `std_msgs/msg/Int32` | Right wheel encoder velocity feedback [counts/s]. |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range measurements from RPLiDAR A1. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D SLAM or static occupancy grid map. |
| `/plan` | `nav_msgs/msg/Path` | Nav2 global planned path to the active goal pose. |

### Subscribed Topics

| Topic | Message Type | Source / Handler |
| :--- | :--- | :--- |
| `/cmd_vel_joy` | `geometry_msgs/msg/Twist` | Mobile Web Dashboard teleoperation joystick (Priority: 100). |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 DWB Controller velocity output (Priority: 10). |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Nav2 autonomous navigation goal dispatched from Web UI. |

### Services & Actions

| Name | Type | Description |
| :--- | :--- | :--- |
| `/map_saver_server/save_map` | `nav2_msgs/srv/SaveMap` | Persists current SLAM occupancy grid to disk (`.yaml` / `.pgm`). |
| `/reinitialize_global_localization` | `std_srvs/srv/Empty` | Resets AMCL particles uniformly across the map space. |
| `/navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | Cancels current autonomous navigation action (Emergency Stop). |

---

## Troubleshooting

1. **Micro-ROS Agent fails to connect (`rmw_uros_ping_agent` times out):**
   - Check the serial cable and port: `ls -l /dev/ttyACM*`.
   - Ensure the user has permissions: `sudo usermod -a -G dialout $USER`.
   - Ensure the baud rate matches: `115200` baud on both ESP32 and launch arguments.
2. **RPLiDAR fails to start (`Cannot bind to the specified serial port`):**
   - Verify udev rules: `ls -l /dev/rplidar`.
   - If not symlinked, provide port directly: `serial_port:=/dev/ttyUSB0`.
3. **Web Dashboard does not connect:**
   - Verify `rosbridge_server` is running on port 9090: `ss -tulpn | grep 9090`.
   - Ensure the mobile device is on the same local network subnet and firewall allows port 9090 (`sudo ufw allow 9090`).
4. **Robot drives backwards or turns in opposite direction:**
   - Invert motor polarity flags in `esp32_test_2/src/main.cpp`: `MOTOR_RIGHT_INVERT` / `MOTOR_LEFT_INVERT` and encoder direction signs `ENC_RIGHT_SIGN` / `ENC_LEFT_SIGN`.

---

## License

This project is licensed under the [MIT License](LICENSE). Academic rights belong to Yıldız Technical University Department of Computer Engineering.
