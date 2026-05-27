# 📡 Topic Specification — 시스템 통신 인터페이스 정의서

> **작성:** Eng A (Tech Lead)
> **승인:** Leader (PM)
> **목적:** 노드 간 통신 규격 통일. 모든 팀원은 이 문서의 Topic 이름·메시지 타입을 그대로 사용한다.

---

## 📌 명명 규칙

1. 모든 Topic은 `/` 로 시작 (절대 경로)
2. 소문자 + 언더스코어 (`/camera/image_raw`)
3. 계층 구조 prefix (`/camera/...`, `/perception/...`)
4. ROS 표준 토픽(`/cmd_vel`, `/scan`, `/odom`, `/imu/data`)은 **이름 변경 금지**
5. 양쪽 환경(`mode:=sim`, `mode:=real`)에서 **동일한 Topic 이름** 사용

---

## 1) 🎮 로봇 제어 (Control)

| Topic          | 메시지 타입                | 방향         | 발행   | 구독  |
|---             |---                        |---           |---    |---    |
| `/cmd_vel`     | `geometry_msgs/Twist`     | FSM → Bridge | Eng B | Eng A |
| `/driving_cmd` | `interfaces/DrivingCmd`   | FSM → Bridge | Eng B | Eng A |
| `/serial/tx`   | `interfaces/SerialPacket` | RPi → ESP32  | Eng A | (HW)  |
| `/serial/rx`   | `interfaces/SerialPacket` | ESP32 → RPi  | Eng A | Eng B |

**비고:** `/cmd_vel`은 Gazebo·표준 호환용. `/driving_cmd`는 우리 커스텀(시리얼 패킷 변환용). 
양쪽 동시 발행 또는 둘 중 하나 선택은 Eng B가 구현 시 결정.

---

## 2) 👁 인식 (Perception)

| Topic                     | 메시지 타입                  | 발행                     | 구독   |
|---                        |---                          |---                       |---     |
| `/camera/image_raw`       | `sensor_msgs/Image`         | Eng C(sim) / Eng A(real) | Leader |
| `/camera/camera_info`     | `sensor_msgs/CameraInfo`    | Eng C / Eng A            | Leader |
| `/perception/detections`  | `interfaces/DetectionArray` | Leader                   | Eng B  |
| `/perception/lane_offset` | `std_msgs/Float32`          | Leader                   | Eng B  |

**비고:** 차선 검출은 OpenCV 기반(YOLO 별도). lane_offset 단위는 미터(m), 양수=우측 이탈, 음수=좌측 이탈.

---

## 3) 📍 위치/자세 (Localization)

| Topic   | 메시지 타입          | 발행                      | 구독   |
|---      |---                  |---                        |---    |
| `/odom` | `nav_msgs/Odometry` | Eng C(sim) / Eng B(real)  | Eng B |

**비고:** sim에서는 Gazebo가 자동 발행. real에서는 ESP32 엔코더 기반 산출.

---

## 4) 🛰 센서 (Sensor)

| Topic       | 메시지 타입              | 발행                     | 구독             |
|---          |---                      |---                       |---               |
| `/scan`     | `sensor_msgs/LaserScan` | Eng C(sim) / Eng A(real) | Eng B (회피)     |
| `/imu/data` | `sensor_msgs/Imu`       | Eng A (real만)           | Eng B (직진 보정) |

**비고:** IMU는 sim에서는 발행 없음 (real에서만 ESP32 → 시리얼 → ROS 변환).

---

## 5) 💚 시스템 상태 (System Health)

| Topic            | 메시지 타입                 | 발행            | 구독           |
|---               |---                         |---             |---              |
| `/robot_status`  | `interfaces/RobotStatus`   | Eng B (FSM)    | Eng C (대시보드) |
| `/system/health` | `interfaces/SystemHealth`  | 전 노드         | Eng A (관제)    |
| `/battery_state` | `sensor_msgs/BatteryState` | Eng A (real만) | Eng C (대시보드) |

**비고:** 각 노드는 1Hz 주기로 `/system/health` 에 자신의 상태 발행. node_name 필드로 구분.

---

## 6) 📐 좌표계 (TF Frames)

| Frame         | 부모         | 발행                     |
|---            |---          |---                       |
| `map`         | (root)      | (없음, 정적)              |
| `odom`        | `map`       | Eng C(sim) / Eng B(real) |
| `base_link`   | `odom`      | Eng B                    |
| `camera_link` | `base_link` | Eng C (URDF)             |
| `laser_link`  | `base_link` | Eng C (URDF)             |

---

## 🧰 메시지 타입별 요약

### 커스텀 (`interfaces/`)

| 메시지            | 용도 |
|---               |---                                                     |
| `Detection`      | YOLO 탐지 단일 객체 (class_id, class_name, conf, bbox)  |
| `DetectionArray` | 한 프레임의 모든 Detection 배열                         |
| `DrivingCmd`     | 주행 명령 (linear/angular velocity, e-stop)            |
| `SerialPacket`   | RPi ↔ ESP32 양방향 패킷 (motor, IMU, status)           |
| `RobotStatus`    | FSM 상태 (state, action, obstacle 감지)                |
| `SystemHealth`   | 노드 하트비트 (node_name, is_alive, error_code)         |

### ROS 표준

| 패키지           | 메시지                                                    |
|---              |---                                                        |
| `geometry_msgs` | `Twist`                                                   |
| `sensor_msgs`   | `Image`, `CameraInfo`, `LaserScan`, `Imu`, `BatteryState` |
| `nav_msgs`      | `Odometry`                                                |
| `std_msgs`      | `Float32`                                                 |

---

## 🚧 변경 절차

1. 변경 제안 → GitHub Issue 또는 Slack
2. Eng A가 검토 + 영향도 분석
3. Leader 승인
4. `TOPIC_SPEC.md` 업데이트 + 메시지 파일(`src/interfaces/msg/`) 수정
5. 영향 받는 노드의 발행자/구독자에게 공지

**원칙:** 한 번 정의된 Topic 이름은 신중히 변경. 가능한 한 필드 추가 위주로 진화.

---

## 📝 변경 이력

| 날짜        | 변경 내용                         | 작성자 |
|---         |---                               |---                |
| 2026-05-21 | 초기 작성 (PM 가이드 + Eng A 정의) | futurexst (Eng A) |