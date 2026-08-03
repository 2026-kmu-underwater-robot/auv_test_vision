# AUV YOLO 비전 반복 테스트

`auv_buoy_vision_control`에서 YOLO 검출기, 모델(`best.pt`), 환경 점검/설치 스크립트, legacy bbox controller를 독립 복사한 패키지입니다. 미션 노드는 Acoustic-Vision 핸드셰이크를 사용하지 않습니다.

노드 시작 즉시 `target_depth_m`까지 잠항하고 아래 상태를 무한 반복합니다.

```text
DIVE -> SEARCH -> TARGET_HOLD -> APPROACH_BUOY -> ALIGN_STICK
                       |                                  |
                       | bbox 유실                         v
                       v                       STRONG_FORWARD -> STRONG_BACKOFF
                REACQUIRE_BUOY                                  |             |
                       |                                         +-------------+--> SEARCH
                       +-- 1초 내 재검출 실패 --> SEARCH
```

수심을 아직 받지 못한 상태에서는 `DIVE`에서 모든 제어축을 중립으로 유지하며 입력을 기다립니다. 마지막 수심은 새 메시지가 들어올 때까지 유지하고, bbox는 1초 동안 새 메시지가 없으면 유실로 처리합니다.

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
  target_depth_m:=1.0 \
  depth_pose_topic:=/depth/pose \
  depth_pose_scale:=-1.0 \
  buoy_class_id:=0 stick_class_id:=1
```

상태와 출력은 다음으로 확인합니다.

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/rc_command
```

## 핵심 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `target_depth_m` | 1.0 | 시작 직후 내려갈 목표 수심(m, positive-down) |
| `search_yaw_pwm` / `search_forward_pwm` | 1530 / 1520 | SEARCH 탐색 명령 |
| `reacquire_yaw_pwm` / `reacquire_yaw_duration_sec` | 1470 / 0.5 | TARGET_HOLD 중 bbox 유실 시 역방향 yaw 재탐색 명령 |
| `reacquire_timeout_sec` | 1.0 | 재탐색 중 bbox가 다시 보이지 않으면 SEARCH로 복귀하는 시간 |
| `approach_area_ratio` | 0.20 | 이 면적비 이상이면 ALIGN 진입 |
| `strong_forward_pwm` / `strong_forward_duration_sec` | 1700 / 0.8 | 정렬 후 강한 전진 |
| `strong_backoff_pwm` / `strong_backoff_duration_sec` | 1300 / 0.8 | 강한 전진 후 강한 후진 |

수심 입력은 `geometry_msgs/msg/PoseWithCovarianceStamped`이고 기본 변환은 `depth = -pose.position.z`입니다. RC 출력은 Ch3 throttle, Ch4 yaw, Ch5 forward이며, 실제 투입 전 PWM 방향과 수심 부호를 반드시 확인해야 합니다.
