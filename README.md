# AUV YOLO 비전 반복 테스트

`auv_buoy_vision_control`에서 YOLO 검출기, 모델(`best.pt`), 환경 점검/설치 스크립트, legacy bbox controller를 독립 복사한 패키지입니다. 미션 노드는 Acoustic-Vision 핸드셰이크 후 반복 비전 테스트를 수행합니다.

노드는 시작 시 RC를 발행하지 않고 Acoustic 요청을 기다립니다. 타깃을 안정적으로 확인해 `/vision/target_confirmed=true`를 발행한 뒤, `/homing/vision_control_granted=true`를 받으면 그 순간의 실제 수심을 캡처하고 최초 1회 `ONCE_SERCH`를 거쳐 아래 테스트를 반복합니다.

```text
IDLE -> TARGET_CONFIRM -> WAIT_CONTROL_GRANT -- 정상 승인 --+
  |                                                       |
  +-- 최초 /homing/timeout=true --------------------------+-> ONCE_SERCH
                                                               | 안정 검출 -> APPROACH_BUOY
                                                               + 10초 종료 -> SEARCH

SEARCH -> TARGET_HOLD -> APPROACH_BUOY -> STRONG_FORWARD -> STRONG_BACKOFF
              |                                                   |
              +-> REACQUIRE_BUOY                                  +-> SEARCH
                       | 1초 내 재검출 실패
                       +----------------------------------------------> SEARCH
```

일반 `SEARCH`는 부표를 찾을 때까지 다음 15초 패턴을 반복합니다. 각 구간에서는 표시된 수평 채널만 구동하고 나머지 yaw/forward 채널은 중립을 유지합니다.

```text
yaw 1560 (10초, 한 바퀴) -> forward 1700 (5초) -> 처음부터 반복
```

어느 구간에서든 부표 bbox가 들어오면 즉시 수평 이동을 중립으로 만들고 `TARGET_HOLD`에서 안정 검출 조건을 확인합니다.

`ONCE_SERCH`는 정상 핸드셰이크 승인과 `/homing/timeout=true` 경로가 같은 1회 플래그를 공유합니다. 먼저 발생한 경로에서 한 번 실행되며, 이후 다른 경로의 신호로 다시 실행되지는 않습니다.

`SEARCH` 진입 후 60초 동안 부표를 찾지 못하면 `SEARCH_RECOVERY`로 전환해 아래 동작을 한 번 수행한 뒤 `SEARCH`로 돌아갑니다. `SEARCH → TARGET_HOLD → REACQUIRE_BUOY → SEARCH` 또는 `SEARCH → TARGET_HOLD → APPROACH_BUOY → SEARCH`로 후보 추적 후 유실된 경우에는 이 60초 누적 타이머를 초기화하지 않습니다. 이 상태에서도 부표를 검출하면 즉시 `TARGET_HOLD`로 전환합니다.

```text
SEARCH_RECOVERY: yaw 1440 (5초) -> forward 1700 (5초) -> SEARCH
```

승인 전에는 RC override와 RELEASE 메시지 모두 발행하지 않습니다. 승인 시 유효한 수심 입력이 없으면 승인을 무시하며, bbox는 1초 동안 새 메시지가 없으면 유실로 처리합니다.

SIGINT, SIGTERM 또는 ROS context shutdown 시에는 pre-shutdown 콜백이 RC RELEASE를 3회 발행하고 전송 확인을 잠시 기다립니다. `SIGKILL`이나 전원 차단처럼 프로세스가 코드를 실행할 수 없는 종료는 노드 내부에서 처리할 수 없으므로 FCU/MAVROS 측 RC override timeout을 함께 설정해야 합니다.

## 빌드

```bash
cd ~/auv_buoy_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select auv_test_vision --symlink-install
source install/setup.bash
```

## 노트북 YOLO

```bash
pip install -r src/auv_test_vision/requirements-yolo.txt
ros2 launch auv_test_vision laptop_yolo_detection.launch.py \
  model_path:=/path/to/model.pt \
  image_topic:=/camera/camera/color/image_raw/compressed \
  publish_per_class:=true
```

기본 모델은 패키지 루트의 `best.pt`이며, 필요하면 `model_path`에 절대 경로를 지정합니다. 부표와 stick을 동시에 사용하므로 `target_class_id:=-1`을 유지합니다.

## AUV 테스트 미션

```bash
ros2 launch auv_test_vision auv_bbox_controller.launch.py \
  depth_pose_topic:=/depth/pose \
  depth_pose_scale:=-1.0 \
  buoy_class_id:=0 stick_class_id:=1
```

핸드셰이크 토픽은 다음 순서로 동작합니다.

```text
/homing/vision_search_active=true   Acoustic -> Vision
/vision/target_confirmed=true       Vision -> Acoustic
/homing/vision_control_granted=true Acoustic -> Vision
/homing/timeout=true                Acoustic -> Vision (최초 1회전 탐색)
```

단독 시험에서는 별도 터미널에서 Bool 메시지를 한 번씩 발행할 수 있습니다.

```bash
ros2 topic pub --once --qos-durability transient_local \
  /homing/vision_search_active std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once --qos-durability transient_local \
  /homing/vision_control_granted std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once --qos-durability transient_local \
  /homing/timeout std_msgs/msg/Bool "{data: true}"
```

상태와 출력은 다음으로 확인합니다.

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/rc_command
ros2 topic echo /vision/target_confirmed
```

## 핵심 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `vision_search_request_topic` | `/homing/vision_search_active` | Acoustic의 비전 탐색 요청 |
| `target_confirmed_topic` | `/vision/target_confirmed` | Vision의 안정 타깃 확인 응답 |
| `vision_control_granted_topic` | `/homing/vision_control_granted` | Acoustic의 제어권 승인 |
| `homing_timeout_topic` | `/homing/timeout` | 최초 timeout 시 `ONCE_SERCH` 진입 |
| `once_search_yaw_pwm` / `once_search_duration_sec` | 1560 / 10.0 | 최초 timeout 시 한 바퀴 탐색 명령 |
| `search_yaw_pwm` / `search_yaw_duration_sec` | 1560 / 10.0 | SEARCH 한 바퀴 회전 구간 |
| `search_forward_pwm` / `search_forward_duration_sec` | 1700 / 5.0 | 회전 중 미검출 시 전진 구간 |
| `search_timeout_sec` | 60.0 | 미검출 시 `SEARCH_RECOVERY`로 넘어가는 시간 |
| `search_recovery_yaw_pwm` / `search_recovery_yaw_duration_sec` | 1440 / 5.0 | 복구 상태의 반대 방향 회전 |
| `search_recovery_forward_pwm` / `search_recovery_forward_duration_sec` | 1700 / 5.0 | 복구 상태의 전진 |
| `reacquire_yaw_pwm` / `reacquire_yaw_duration_sec` | 1470 / 0.5 | TARGET_HOLD 중 bbox 유실 시 역방향 yaw 재탐색 명령 |
| `reacquire_timeout_sec` | 1.0 | 재탐색 중 bbox가 다시 보이지 않으면 SEARCH로 복귀하는 시간 |
| `yaw_kp_pwm` | 110.0 | 정규화 수평 오차 1.0당 yaw PWM 비례 gain |
| `max_yaw_delta` | 100 | 중립값에서 허용할 yaw PWM 최대 변화량 |
| `approach_area_ratio` | 0.10 | 이 면적비 이상이면 전진 중립으로 근접 정렬 유지 시작 |
| `buoy_align_target_x` / `buoy_align_target_y` | 0.50 / 0.50 | APPROACH가 계속 추종하는 화면 중앙 bbox 목표 좌표 |
| `approach_close_hold_sec` | 2.0 | 목표 면적 이상을 연속 유지한 뒤 STRONG_FORWARD로 넘어가는 시간 |
| `strong_forward_pwm` / `strong_forward_duration_sec` | 1700 / 1.2 | 정렬 후 강한 전진 |
| `strong_backoff_pwm` / `strong_backoff_duration_sec` | 1300 / 1.2 | 강한 전진 후 강한 후진 |

수심 입력은 `geometry_msgs/msg/PoseWithCovarianceStamped`이고 기본 변환은 `depth = -pose.position.z`입니다. RC 출력은 Ch3 throttle, Ch4 yaw, Ch5 forward이며, 실제 투입 전 PWM 방향과 수심 부호를 반드시 확인해야 합니다.

급회전이 크면 다음처럼 gain과 최대 변화량을 낮춰 실행할 수 있습니다.

```bash
ros2 launch auv_test_vision auv_bbox_controller.launch.py \
  yaw_kp_pwm:=70.0 max_yaw_delta:=90
```
