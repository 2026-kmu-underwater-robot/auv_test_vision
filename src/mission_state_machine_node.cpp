// AUV 부표(buoy) 미션 상태머신 노드.
//
// 비전 bbox + 수심을 받아 MAVROS RC override로 throttle/yaw/forward를 제어한다.
// 테스트 흐름:
//   DIVE -> SEARCH -> APPROACH_BUOY -> ALIGN_STICK -> STRONG_FORWARD
//        -> STRONG_BACKOFF -> SEARCH (무한 반복)

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class MissionStateMachineNode : public rclcpp::Node
{
public:
  MissionStateMachineNode()
  : Node("mission_state_machine_node")
  {
    // --- 토픽 ---
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    // pose.z 를 양의 하방(positive-down) 수심[m]으로 변환: depth = scale * z + offset
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", -1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    state_topic_ = declare_parameter<std::string>("state_topic", "/mission/state");
    rc_override_topic_ =
      declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    rc_monitor_topic_ =
      declare_parameter<std::string>("rc_monitor_topic", "/mission/rc_command");

    // --- 제어 주기 / 타임아웃 / 수심 ---
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 1.0);
    surface_depth_m_ = declare_parameter<double>("surface_depth_m", 0.4);
    // 핸드셰이크 인계 수심 대신, 노드 시작 시 이 목표 수심으로 즉시 잠항한다.
    target_depth_m_ = declare_parameter<double>("target_depth_m", 1.0);
    depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.2);
    // Hydrophone 제어기와 같은 PID 구조. 수심은 positive-down이므로 양의 오차는 하강 명령이다.
    depth_kp_ = declare_parameter<double>("depth_kp", 0.3);
    depth_ki_ = declare_parameter<double>("depth_ki", 0.05);
    depth_kd_ = declare_parameter<double>("depth_kd", 0.04);
    depth_bias_ = declare_parameter<double>("depth_bias", 0.1);
    depth_integral_limit_ = declare_parameter<double>("depth_integral_limit", 2.0);
    heave_limit_ = declare_parameter<double>("heave_limit", 0.4);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 160);
    // 수심·bbox 중심/크기 1차 LPF 시상수[s]. 0이면 필터 비활성
    lpf_tau_sec_ = declare_parameter<double>("lpf_tau_sec", 0.3);

    // --- 비전 탐지 / 탐색 ---
    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    min_detection_hits_ = declare_parameter<int>("min_detection_hits", 3);
    target_confirm_hits_ = declare_parameter<int>("target_confirm_hits", 3);
    target_confirm_sec_ = declare_parameter<double>("target_confirm_sec", 0.2);
    // bbox 면적 / 이미지 면적 비율이 이 값 이상이면 "충분히 가까움"으로 판단
    approach_area_ratio_ = declare_parameter<double>("approach_area_ratio", 0.20);
    search_timeout_sec_ = declare_parameter<double>("search_timeout_sec", 40.0);
    area_verify_sec_ = declare_parameter<double>("area_verify_sec", 12.0);
    // SEARCH 다중 부표 선택: 면적 비슷 판정 / confidence 비슷 판정 / 동일 타깃 판정
    buoy_area_similar_ratio_ = declare_parameter<double>("buoy_area_similar_ratio", 0.15);
    buoy_confidence_similar_delta_ =
      declare_parameter<double>("buoy_confidence_similar_delta", 0.05);
    buoy_same_target_center_ratio_ =
      declare_parameter<double>("buoy_same_target_center_ratio", 0.12);

    // --- 막대(stick) 정렬 ---
    // 이미지 3사분면(왼쪽 아래) 쪽. 끝단이 아닌 대략 (0.30, 0.70)
    fork_target_x_ = declare_parameter<double>("fork_target_x", 0.30);
    fork_target_y_ = declare_parameter<double>("fork_target_y", 0.70);
    // buoy 중심이 이 경계를 넘으면 이미지 오른쪽 위 허용 영역으로 판단한다.
    // x는 오른쪽으로, y는 아래쪽으로 증가한다.
    align_zone_min_x_ = declare_parameter<double>("align_zone_min_x", 0.60);
    align_zone_max_y_ = declare_parameter<double>("align_zone_max_y", 0.40);
    align_stable_sec_ = declare_parameter<double>("align_stable_sec", 0.7);

    // --- 포크 삽입 / 분리 / 후퇴 / 검증 ---
    insert_pwm_ = declare_parameter<int>("insert_pwm", 1560);
    insert_duration_sec_ = declare_parameter<double>("insert_duration_sec", 0.8);
    detach_pwm_ = declare_parameter<int>("detach_pwm", 1620);
    detach_duration_sec_ = declare_parameter<double>("detach_duration_sec", 0.3);
    backoff_pwm_ = declare_parameter<int>("backoff_pwm", 1420);
    backoff_duration_sec_ = declare_parameter<double>("backoff_duration_sec", 0.5);
    // 반복 시험용 강한 전진/후진 명령. 기존 삽입/분리/후퇴 파라미터는 호환을 위해 유지한다.
    strong_forward_pwm_ = declare_parameter<int>("strong_forward_pwm", 1700);
    strong_forward_duration_sec_ = declare_parameter<double>("strong_forward_duration_sec", 0.8);
    strong_backoff_pwm_ = declare_parameter<int>("strong_backoff_pwm", 1300);
    strong_backoff_duration_sec_ = declare_parameter<double>("strong_backoff_duration_sec", 0.8);
    verify_clear_sec_ = declare_parameter<double>("verify_clear_sec", 1.0);
    verify_timeout_sec_ = declare_parameter<double>("verify_timeout_sec", 3.0);
    max_target_retries_ = declare_parameter<int>("max_target_retries", 2);

    // --- RC 채널 / PWM ---
    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    max_yaw_delta_ = declare_parameter<int>("max_yaw_delta", 180);
    max_tracking_depth_delta_ = declare_parameter<int>("max_tracking_depth_delta", 100);
    // APPROACH 전진 최대/최소 PWM. 멀리서 max, 가까워질수록 min까지 선형(P) 감속
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1700);
    approach_forward_min_pwm_ = declare_parameter<int>("approach_forward_min_pwm", 1560);
    // APPROACH/ALIGN throttle: 1=비전만, 0=수심 P만. 기본 0.4는 수심 쪽에 조금 더 무게
    approach_vision_throttle_weight_ =
      declare_parameter<double>("approach_vision_throttle_weight", 0.4);
    search_yaw_pwm_ = declare_parameter<int>("search_yaw_pwm", 1530);
    search_forward_pwm_ = declare_parameter<int>("search_forward_pwm", 1520);
    reacquire_yaw_pwm_ = declare_parameter<int>("reacquire_yaw_pwm", 1470);
    reacquire_yaw_duration_sec_ =
      declare_parameter<double>("reacquire_yaw_duration_sec", 0.5);
    reacquire_timeout_sec_ = declare_parameter<double>("reacquire_timeout_sec", 1.0);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);
    // true면 throttle PWM 증가 = 상승 (일반적인 설정)
    vertical_positive_is_up_ = declare_parameter<bool>("vertical_positive_is_up", true);

    validate_parameters();

    // --- 구독 / 발행 ---
    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10, std::bind(&MissionStateMachineNode::on_bbox, this, std::placeholders::_1));
    depth_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      depth_pose_topic_, 10,
      std::bind(&MissionStateMachineNode::on_depth_pose, this, std::placeholders::_1));
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    rc_monitor_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_monitor_topic_, 10);
    // latched: 늦게 구독해도 마지막 상태를 받을 수 있음
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());

    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&MissionStateMachineNode::on_timer, this));

    state_entered_at_ = now();
    mission_hold_depth_m_ = target_depth_m_;
    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "Looping vision test ready; target_depth=%.2f m depth_pose=%s bbox=%s state=%s "
      "rc_output=%s rc_monitor=%s", target_depth_m_, depth_pose_topic_.c_str(), bbox_topic_.c_str(),
      state_topic_.c_str(), rc_override_topic_.c_str(), rc_monitor_topic_.c_str());
  }

  // 노드 종료 시 제어 채널을 한 번 RELEASE 해서 수동/다른 제어기에 넘긴다.
  void publish_release_once()
  {
    auto channels = nochange_channels();
    release_controlled_channels(channels);
    publish_channels(channels);
  }

private:
  // 미션 단계. 타이머 콜백에서 switch로 분기한다.
  enum class State
  {
    DIVE,            // 시작 즉시 target_depth_m으로 잠항
    SEARCH,          // yaw 회전하며 buoy 탐색
    TARGET_HOLD,     // buoy 검출 즉시 수평 중립으로 정지하고 안정 검출 확인
    REACQUIRE_BUOY,  // target hold 중 유실된 buoy를 짧게 역방향 yaw로 재탐색
    APPROACH_BUOY,   // buoy 중심 추적 + 전진
    ALIGN_STICK,     // stick을 포크 목표점으로 정밀 정렬
    STRONG_FORWARD,  // 강한 전진 펄스
    STRONG_BACKOFF,  // 강한 후진 펄스 후 SEARCH로 반복
    // 아래 상태들은 원본의 제어 알고리즘/호환 파라미터를 보존한다. 이 테스트 흐름에서는 진입하지 않는다.
    INSERT_FORK,
    DETACH,
    BACKOFF,
    VERIFY_RELEASE,  // 대상이 사라졌는지 확인 (재시도 가능)
    AREA_VERIFY,     // 최종 영역 재탐색
    ASCEND,          // 수면으로 부상
    COMPLETE         // 미션 완료
  };

  // bbox 토픽 Float32MultiArray 레이아웃 (index):
  //   [1]=유효성(>=0.5), [2]=class_id, [3]=confidence,
  //   [4]=cx, [5]=cy, [6]=w, [7]=h, [8]=img_w, [9]=img_h
  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{0};  // 타임아웃 내 연속 수신 횟수
  };

  void validate_parameters()
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "Thruster PWM range must stay within 1300..1700 and include neutral_pwm");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("RC channel numbers must be in [1, 18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("Controlled RC channels must be unique");
    }
    if (depth_pose_topic_.empty()) {
      throw std::invalid_argument("depth_pose_topic must not be empty");
    }
    if (buoy_class_id_ == stick_class_id_) {
      throw std::invalid_argument("buoy_class_id and stick_class_id must differ");
    }
    if (approach_vision_throttle_weight_ < 0.0 || approach_vision_throttle_weight_ > 1.0) {
      throw std::invalid_argument("approach_vision_throttle_weight must be in [0, 1]");
    }
    if (lpf_tau_sec_ < 0.0) {
      throw std::invalid_argument("lpf_tau_sec must be >= 0");
    }
    if (reacquire_yaw_duration_sec_ < 0.0 || reacquire_timeout_sec_ < reacquire_yaw_duration_sec_) {
      throw std::invalid_argument("invalid reacquire timing parameters");
    }
    if (
      !std::isfinite(depth_kp_) || !std::isfinite(depth_ki_) ||
      !std::isfinite(depth_kd_) || !std::isfinite(depth_bias_) ||
      !std::isfinite(depth_integral_limit_) || !std::isfinite(heave_limit_) ||
      depth_kp_ < 0.0 || depth_ki_ < 0.0 || depth_kd_ < 0.0 ||
      depth_integral_limit_ < 0.0 || heave_limit_ <= 0.0 ||
      max_depth_delta_pwm_ < 0)
    {
      throw std::invalid_argument("invalid depth PID parameters");
    }
    if (target_confirm_hits_ < 1 || target_confirm_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid acoustic-vision handshake confirmation parameters");
    }
    if (
      !std::isfinite(align_zone_min_x_) || !std::isfinite(align_zone_max_y_) ||
      align_zone_min_x_ < 0.0 || align_zone_min_x_ > 1.0 ||
      align_zone_max_y_ < 0.0 || align_zone_max_y_ > 1.0)
    {
      throw std::invalid_argument("alignment zone bounds must be in [0, 1]");
    }
  }

  void on_depth_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    accept_depth(depth_pose_scale_ * msg->pose.pose.position.z + depth_pose_offset_m_);
  }

  // 수심은 양의 하방[m]. 비정상 값은 무시한다. 연속 샘플은 LPF로 완화.
  void accept_depth(double depth_m)
  {
    if (!std::isfinite(depth_m) || depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid depth value");
      return;
    }
    const auto received_at = now();
    if (
      depth_m_)
    {
      const double dt = (received_at - depth_received_at_).seconds();
      depth_m_ = low_pass(*depth_m_, depth_m, dt);
    } else {
      depth_m_ = depth_m;
    }
    depth_received_at_ = received_at;
  }

  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 메시지에 10개 단위 블록이 여러 개 올 수 있음 (한 프레임 다중 검출)
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox with fewer than 10 values");
      return;
    }

    std::optional<Detection> best_buoy;
    std::optional<Detection> best_stick;
    const size_t block_count = msg->data.size() / 10;
    for (size_t block = 0; block < block_count; ++block) {
      const size_t base = block * 10;
      bool finite = true;
      for (size_t index = 0; index < 10; ++index) {
        if (!std::isfinite(msg->data[base + index])) {
          finite = false;
          break;
        }
      }
      if (!finite) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox containing NaN/Inf");
        continue;
      }
      // [1]<0.5 또는 이미지 크기 비정상이면 미탐지로 무시
      if (
        msg->data[base + 1] < 0.5F || msg->data[base + 8] <= 0.0F ||
        msg->data[base + 9] <= 0.0F)
      {
        continue;
      }

      Detection det{
        msg->data[base + 3], msg->data[base + 4], msg->data[base + 5],
        msg->data[base + 6], msg->data[base + 7], msg->data[base + 8],
        msg->data[base + 9], now(), 1};
      const int class_id = static_cast<int>(std::lround(msg->data[base + 2]));
      if (class_id == buoy_class_id_) {
        // [ACOUSTIC-VISION HANDOFF V2] YOLO와 동일하게 면적 우선으로 가까운 buoy를 선택한다.
        if (!best_buoy || is_better_buoy(det, *best_buoy)) {
          best_buoy = det;
        }
      } else if (class_id == stick_class_id_) {
        if (!best_stick || det.confidence > best_stick->confidence) {
          best_stick = det;
        }
      }
    }

    if (best_buoy) {
      accept_buoy_detection(*best_buoy);
      // 검출 콜백에서 즉시 정지 상태를 예약해 다음 제어 주기부터 수평 이동을 멈춘다.
      if (state_ == State::SEARCH) {
        transition_to(State::TARGET_HOLD, "buoy detected; horizontal motion stopped");
      }
    }
    if (best_stick) {
      update_detection_slot(stick_, *best_stick);
    }
  }

  // SEARCH/AREA_VERIFY: 면적 > 박스 확률(confidence) > 오른쪽 순으로 타깃 선택.
  // APPROACH/ALIGN: 같은 타깃만 갱신(탐색 중 고른 부표를 유지).
  void accept_buoy_detection(Detection incoming)
  {
    const bool selecting =
      state_ == State::SEARCH || state_ == State::AREA_VERIFY ||
      state_ == State::DIVE || state_ == State::TARGET_HOLD ||
      state_ == State::REACQUIRE_BUOY;
    if (!recent(buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      return;
    }
    if (same_buoy_target(incoming, *buoy_)) {
      update_detection_slot(buoy_, incoming);
      return;
    }
    if (selecting && is_better_buoy(incoming, *buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      return;
    }
    // APPROACH 등에서는 다른 부표로 타깃을 바꾸지 않음
  }

  bool same_buoy_target(const Detection & a, const Detection & b) const
  {
    const double ref_w = std::max(a.image_width, b.image_width);
    const double ref_h = std::max(a.image_height, b.image_height);
    if (ref_w <= 0.0 || ref_h <= 0.0) {
      return false;
    }
    const double dx = std::abs(a.center_x - b.center_x) / ref_w;
    const double dy = std::abs(a.center_y - b.center_y) / ref_h;
    return dx <= buoy_same_target_center_ratio_ && dy <= buoy_same_target_center_ratio_;
  }

  // 면적 큰 것 우선. 비슷하면 박스 확률(confidence) 높은 것. 그것도 비슷하면 오른쪽.
  bool is_better_buoy(const Detection & candidate, const Detection & current) const
  {
    const double cand_area = std::max(0.0, static_cast<double>(candidate.width * candidate.height));
    const double cur_area = std::max(0.0, static_cast<double>(current.width * current.height));
    const double larger = std::max({cand_area, cur_area, 1.0});
    if (std::abs(cand_area - cur_area) > buoy_area_similar_ratio_ * larger) {
      return cand_area > cur_area;
    }
    if (
      std::abs(candidate.confidence - current.confidence) > buoy_confidence_similar_delta_)
    {
      return candidate.confidence > current.confidence;
    }
    return candidate.center_x > current.center_x;
  }

  // 최근 탐지와 이어지면 consecutive_hits를 증가시키고 중심/크기에 LPF를 적용한다.
  // 타깃이 바뀌거나 타임아웃 후에는 raw 값으로 리셋.
  void update_detection_slot(std::optional<Detection> & slot, Detection incoming)
  {
    const auto received_at = now();
    int hits = 1;
    if (slot && (received_at - slot->received_at).seconds() <= detection_timeout_sec_) {
      hits = slot->consecutive_hits + 1;
      const double dt = (received_at - slot->received_at).seconds();
      incoming.center_x = static_cast<float>(low_pass(slot->center_x, incoming.center_x, dt));
      incoming.center_y = static_cast<float>(low_pass(slot->center_y, incoming.center_y, dt));
      incoming.width = static_cast<float>(low_pass(slot->width, incoming.width, dt));
      incoming.height = static_cast<float>(low_pass(slot->height, incoming.height, dt));
    }
    incoming.received_at = received_at;
    incoming.consecutive_hits = hits;
    slot = incoming;
  }

  // 1차 LPF: y = αx + (1-α)y_prev, α = dt/(τ+dt). τ=0이면 필터 없음.
  double low_pass(double previous, double sample, double dt_sec) const
  {
    if (lpf_tau_sec_ <= 1e-9 || dt_sec <= 0.0) {
      return sample;
    }
    const double alpha = dt_sec / (lpf_tau_sec_ + dt_sec);
    return alpha * sample + (1.0 - alpha) * previous;
  }

  // 주기 제어 루프: enable/수심 가드 후 현재 State별 동작을 수행하고 RC를 발행한다.
  void on_timer()
  {
    auto channels = nochange_channels();

    switch (state_) {
      case State::DIVE:
        run_dive(channels);
        break;
      case State::SEARCH:
        run_search(channels);
        break;
      case State::TARGET_HOLD:
        run_target_hold(channels);
        break;
      case State::REACQUIRE_BUOY:
        run_reacquire_buoy(channels);
        break;
      case State::APPROACH_BUOY:
        run_approach(channels);
        break;
      case State::ALIGN_STICK:
        run_align_stick(channels);
        break;
      case State::STRONG_FORWARD:
        // 원본과 동일한 작업수심 PID를 유지하면서 강한 전진.
        set_neutral_control(channels);
        hold_work_depth(channels);
        if (state_age_sec() >= strong_forward_duration_sec_) {
          transition_to(State::STRONG_BACKOFF, "strong forward pulse complete");
          set_channel(channels, forward_channel_, strong_backoff_pwm_);
        } else {
          set_channel(channels, forward_channel_, strong_forward_pwm_);
        }
        break;
      case State::STRONG_BACKOFF:
        // 원본과 동일한 작업수심 PID를 유지하면서 강한 후진 후 즉시 반복.
        set_neutral_control(channels);
        hold_work_depth(channels);
        if (state_age_sec() >= strong_backoff_duration_sec_) {
          buoy_.reset();
          stick_.reset();
          target_retries_ = 0;
          transition_to(State::SEARCH, "strong backoff complete; loop restarted");
        } else {
          set_channel(channels, forward_channel_, strong_backoff_pwm_);
        }
        break;
      // 원본 상태 호환 경로(현재 반복 테스트 전이에서는 사용하지 않음).
      case State::INSERT_FORK:
        set_neutral_control(channels);
        hold_work_depth(channels);
        set_channel(channels, forward_channel_, insert_pwm_);
        if (state_age_sec() >= insert_duration_sec_) {
          transition_to(State::DETACH, "fork insertion pulse complete");
        }
        break;
      case State::DETACH:
        set_neutral_control(channels);
        hold_work_depth(channels);
        set_channel(channels, forward_channel_, detach_pwm_);
        if (state_age_sec() >= detach_duration_sec_) {
          transition_to(State::BACKOFF, "detach pulse complete");
        }
        break;
      case State::BACKOFF:
        set_neutral_control(channels);
        hold_work_depth(channels);
        set_channel(channels, forward_channel_, backoff_pwm_);
        if (state_age_sec() >= backoff_duration_sec_) {
          transition_to(State::VERIFY_RELEASE, "backoff complete");
        }
        break;
      case State::VERIFY_RELEASE:
        run_verify_release(channels);
        break;
      case State::AREA_VERIFY:
        run_area_verify(channels);
        break;
      case State::ASCEND:
        run_ascend(channels);
        break;
      case State::COMPLETE:
        release_controlled_channels(channels);
        break;
    }

    publish_channels(channels);
  }

  // 시작 직후 설정된 작업수심으로 잠항하고, 허용 오차 안이면 탐색을 시작한다.
  void run_dive(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    // 최초 수심 수신 전에는 모든 제어축을 중립으로 유지한다.
    if (!depth_m_ || !mission_hold_depth_m_) {
      return;
    }
    hold_work_depth(channels);
    if (std::abs(*depth_m_ - *mission_hold_depth_m_) <= depth_tolerance_m_) {
      transition_to(State::SEARCH, "target depth reached");
    }
  }

  // 원본 TARGET_CONFIRM의 동일한 연속 hit + 유지시간 검증을 TARGET_HOLD에서 수행한다.
  // Acoustic에 확인을 발행하거나 grant를 기다리는 부분만 제거했다.
  void run_search_confirm()
  {
    const bool stable_candidate = recent(buoy_) &&
      buoy_->consecutive_hits >= target_confirm_hits_;
    if (!stable_candidate) {
      target_confirm_started_at_.reset();
      return;
    }
    if (!target_confirm_started_at_) {
      target_confirm_started_at_ = now();
      return;
    }
    if ((now() - *target_confirm_started_at_).seconds() >= target_confirm_sec_) {
      transition_to(State::APPROACH_BUOY, "stable nearest buoy confirmed");
    }
  }

  // 수심 유지 + 완만한 전진/yaw 회전 탐색. buoy를 발견하면 즉시 TARGET_HOLD로 전환한다.
  void run_search(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
    set_channel(channels, yaw_channel_, search_yaw_pwm_);
    set_channel(channels, forward_channel_, search_forward_pwm_);
    if (recent(buoy_)) {
      transition_to(State::TARGET_HOLD, "buoy detected; horizontal motion stopped");
    }
    // SEARCH timeout을 두지 않는다. 반복 테스트는 탐색을 계속한다.
  }

  // buoy를 발견한 순간 yaw/forward를 중립으로 두고 안정 검출을 확인한다.
  void run_target_hold(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_work_depth(channels);
    if (!recent(buoy_)) {
      buoy_.reset();
      stick_.reset();
      target_confirm_started_at_.reset();
      transition_to(State::REACQUIRE_BUOY, "buoy lost while stopped");
      return;
    }
    run_search_confirm();
  }

  // 유실 직후 처음 0.5초만 yaw=1470으로 역방향 탐색하고, 총 1초 동안 bbox 재검출을 기다린다.
  void run_reacquire_buoy(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_work_depth(channels);
    if (state_age_sec() < reacquire_yaw_duration_sec_) {
      set_channel(channels, yaw_channel_, reacquire_yaw_pwm_);
    }
    if (recent(buoy_)) {
      transition_to(State::TARGET_HOLD, "buoy reacquired");
      return;
    }
    if (state_age_sec() >= reacquire_timeout_sec_) {
      stick_.reset();
      target_confirm_started_at_.reset();
      transition_to(State::SEARCH, "buoy was not reacquired within timeout");
    }
  }

  // buoy를 화면 중심으로 추적하며 전진(면적 기반 P).
  // throttle은 비전 상하 + 작업수심 P를 블렌딩. 충분히 가까우면 stick 검출 없이 ALIGN.
  void run_approach(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      set_neutral_control(channels);
      transition_to(State::SEARCH, "buoy lost during approach");
      return;
    }
    apply_visual_tracking(
      channels, *buoy_, 0.5, 0.5, approach_forward_pwm_from_area(*buoy_),
      *mission_hold_depth_m_, approach_vision_throttle_weight_);
    if (detection_area_ratio(*buoy_) >= approach_area_ratio_) {
      transition_to(State::ALIGN_STICK, "close buoy reached approach area ratio");
    }
  }

  // 박스 면적 비율로 전진 P제어 (선형 감속).
  // ratio≈0(멀리): approach_forward_pwm_(1700)
  // ratio→approach_area_ratio_(가깝게): approach_forward_min_pwm_(1560)
  int approach_forward_pwm_from_area(const Detection & buoy) const
  {
    const double ratio = detection_area_ratio(buoy);
    const double error = std::max(0.0, approach_area_ratio_ - ratio);
    const int floor_pwm = std::min(approach_forward_min_pwm_, approach_forward_pwm_);
    const int max_delta = std::max(0, approach_forward_pwm_ - floor_pwm);
    const double kp =
      (approach_area_ratio_ > 1e-6) ? (static_cast<double>(max_delta) / approach_area_ratio_) : 0.0;
    const int delta = static_cast<int>(std::lround(kp * error));
    return std::clamp(floor_pwm + delta, floor_pwm, approach_forward_pwm_);
  }

  // buoy 중심을 이미지 오른쪽 위 허용 영역으로 보낸다.
  // 영역 안에서는 해당 축을 중립으로 두고, align_stable_sec_ 유지 시 STRONG_FORWARD로 전이한다.
  void run_align_stick(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      set_neutral_control(channels);
      transition_to(State::SEARCH, "buoy lost during fine alignment");
      return;
    }
    const double x = buoy_->center_x / buoy_->image_width;
    const double y = buoy_->center_y / buoy_->image_height;
    const bool in_alignment_zone = x >= align_zone_min_x_ && y <= align_zone_max_y_;

    // 이미 만족한 축은 보정하지 않는다. 관성으로 한 축이 다시 밀리는 것을 줄인다.
    const double error_x = x < align_zone_min_x_
      ? std::clamp((x - align_zone_min_x_) * 2.0, -1.0, 0.0) : 0.0;
    const double error_y = y > align_zone_max_y_
      ? std::clamp((y - align_zone_max_y_) * 2.0, 0.0, 1.0) : 0.0;
    // 정렬 중에도 수심 P를 섞어 양성 부력으로 뜨는 것을 막는다.
    apply_tracking_errors(
      channels, error_x, error_y, neutral_pwm_, *mission_hold_depth_m_,
      approach_vision_throttle_weight_);

    if (in_alignment_zone) {
      if (!condition_started_at_) {
        condition_started_at_ = now();
      } else if ((now() - *condition_started_at_).seconds() >= align_stable_sec_) {
        transition_to(State::STRONG_FORWARD, "alignment zone stable");
      }
    } else {
      condition_started_at_.reset();
    }
  }

  // 후퇴 후 buoy가 사라지면 성공으로 SEARCH 복귀. 남아 있으면 재시도 또는 포기.
  void run_verify_release(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_work_depth(channels);
    if (!recent(buoy_) && state_age_sec() >= verify_clear_sec_) {
      target_retries_ = 0;
      buoy_.reset();
      stick_.reset();
      transition_to(State::SEARCH, "target absent after backoff; provisional success");
      return;
    }
    if (state_age_sec() >= verify_timeout_sec_) {
      if (target_retries_ < max_target_retries_) {
        ++target_retries_;
        transition_to(
          recent(stick_) ? State::ALIGN_STICK : State::APPROACH_BUOY,
          "target remains; retrying detach");
      } else {
        RCLCPP_ERROR(get_logger(), "Target retry limit reached; abandoning this target");
        target_retries_ = 0;
        buoy_.reset();
        stick_.reset();
        transition_to(State::SEARCH, "target retry limit reached");
      }
    }
  }

  // 마지막 재탐색. buoy 발견 시 APPROACH, 없으면 ASCEND.
  void run_area_verify(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
    set_channel(channels, yaw_channel_, search_yaw_pwm_);
    set_channel(channels, forward_channel_, search_forward_pwm_);
    if (confirmed_buoy()) {
      transition_to(State::APPROACH_BUOY, "buoy found during area verification");
    } else if (state_age_sec() >= area_verify_sec_) {
      transition_to(State::ASCEND, "area verification complete with no targets");
    }
  }

  // 수면 근처까지 부상 후 COMPLETE.
  void run_ascend(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    // 부상 중에는 양성 부력 바이어스를 끄고 수면 목표만 추종
    set_channel(channels, throttle_channel_, depth_control_pwm(surface_depth_m_, false));
    if (*depth_m_ <= surface_depth_m_ + depth_tolerance_m_) {
      transition_to(State::COMPLETE, "surface depth reached");
    }
  }

  // 화면 정규화 오차로 yaw/throttle 보정 + forward 설정.
  // depth_blend_target_m 이 있으면 throttle = w*비전 + (1-w)*수심P.
  // error_x/y 는 [-1, 1], 목표점 기준 화면 중심 대비 편차.
  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    double target_x, double target_y, int forward_pwm,
    std::optional<double> depth_blend_target_m = std::nullopt,
    double vision_throttle_weight = 1.0)
  {
    const auto [error_x, error_y] = normalized_error(detection, target_x, target_y);
    apply_tracking_errors(
      channels, error_x, error_y, forward_pwm, depth_blend_target_m, vision_throttle_weight);
  }

  void apply_tracking_errors(
    std::array<uint16_t, 18> & channels, double error_x, double error_y, int forward_pwm,
    std::optional<double> depth_blend_target_m = std::nullopt,
    double vision_throttle_weight = 1.0)
  {
    set_neutral_control(channels);
    const double yaw_sign = yaw_invert_ ? -1.0 : 1.0;
    const double vertical_sign = vertical_positive_is_up_ ? -1.0 : 1.0;
    set_channel(
      channels, yaw_channel_,
      neutral_pwm_ + static_cast<int>(yaw_sign * error_x * max_yaw_delta_));

    const int vision_throttle =
      neutral_pwm_ + static_cast<int>(vertical_sign * error_y * max_tracking_depth_delta_);
    int throttle_pwm = vision_throttle;
    if (depth_blend_target_m && depth_m_) {
      const int depth_throttle = depth_control_pwm(*depth_blend_target_m);
      const double w = std::clamp(vision_throttle_weight, 0.0, 1.0);
      throttle_pwm = static_cast<int>(std::lround(
        w * static_cast<double>(vision_throttle) +
        (1.0 - w) * static_cast<double>(depth_throttle)));
    }
    set_channel(channels, throttle_channel_, throttle_pwm);
    set_channel(channels, forward_channel_, forward_pwm);
  }

  // 탐지 중심을 [0,1]로 정규화한 뒤 목표점 대비 오차를 [-1,1]로 클램프.
  std::pair<double, double> normalized_error(
    const Detection & detection, double target_x, double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp((x - target_x) * 2.0, -1.0, 1.0),
      std::clamp((y - target_y) * 2.0, -1.0, 1.0)};
  }

  double detection_area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  // Hydrophone 인계 시 저장한 작업수심 유지용 throttle 설정.
  void hold_work_depth(std::array<uint16_t, 18> & channels)
  {
    if (!depth_m_ || !mission_hold_depth_m_) {
      return;
    }
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
  }

  // Hydrophone 제어기와 같은 PID + anti-windup. 목표 수심은 인계 시 캡처한 값으로 유지한다.
  // 수심은 positive-down이라 target-current가 양수면 하강 명령을 낸다.
  int depth_control_pwm(double target_depth_m, bool apply_depth_bias = true)
  {
    const double error_m = target_depth_m - *depth_m_;
    const auto current_time = now();
    double dt = 0.0;
    double error_derivative = 0.0;
    if (depth_pid_initialized_) {
      dt = std::clamp((current_time - last_depth_control_time_).seconds(), 0.0, 0.2);
      if (dt > 1e-6) {
        error_derivative = (error_m - previous_depth_error_m_) / dt;
      }
    }
    last_depth_control_time_ = current_time;
    previous_depth_error_m_ = error_m;
    depth_pid_initialized_ = true;

    const double candidate_integral = std::clamp(
      depth_error_integral_ + error_m * dt,
      -depth_integral_limit_, depth_integral_limit_);
    const double bias = apply_depth_bias ? depth_bias_ : 0.0;
    const double candidate_heave = bias + depth_kp_ * error_m +
      depth_ki_ * candidate_integral + depth_kd_ * error_derivative;
    // 포화 상태에서 더 포화시키는 방향으로는 적분하지 않는다.
    if (
      std::abs(candidate_heave) <= heave_limit_ ||
      (candidate_heave > heave_limit_ && error_m < 0.0) ||
      (candidate_heave < -heave_limit_ && error_m > 0.0))
    {
      depth_error_integral_ = candidate_integral;
    }
    const double heave = std::clamp(
      bias + depth_kp_ * error_m + depth_ki_ * depth_error_integral_ +
      depth_kd_ * error_derivative,
      -heave_limit_, heave_limit_);
    const double pwm_scale = static_cast<double>(max_depth_delta_pwm_) / heave_limit_;
    const int delta = static_cast<int>(std::lround(heave * pwm_scale));
    const int sign = vertical_positive_is_up_ ? -1 : 1;
    return neutral_pwm_ + sign * delta;
  }

  void reset_depth_pid()
  {
    depth_pid_initialized_ = false;
    previous_depth_error_m_ = 0.0;
    depth_error_integral_ = 0.0;
    last_depth_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool recent(const std::optional<Detection> & detection) const
  {
    return detection && (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  // 최근 탐지 + 연속 hit 수가 충분할 때만 buoy를 "확정"한다 (오탐 완화).
  bool confirmed_buoy() const
  {
    return recent(buoy_) && buoy_->consecutive_hits >= min_detection_hits_;
  }

  double state_age_sec() const
  {
    return (now() - state_entered_at_).seconds();
  }

  // 상태 전이. 동일 상태면 no-op. 진입 시각·조건 타이머를 리셋하고 state 토픽을 발행한다.
  void transition_to(State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    condition_started_at_.reset();
    if (next == State::ASCEND || next == State::COMPLETE) {
      reset_depth_pid();
    }
    if (next == State::COMPLETE) {
      mission_hold_depth_m_.reset();
    }
    publish_state();
  }

  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::DIVE: return "DIVE";
      case State::SEARCH: return "SEARCH";
      case State::TARGET_HOLD: return "TARGET_HOLD";
      case State::REACQUIRE_BUOY: return "REACQUIRE_BUOY";
      case State::APPROACH_BUOY: return "APPROACH_BUOY";
      case State::ALIGN_STICK: return "ALIGN_STICK";
      case State::STRONG_FORWARD: return "STRONG_FORWARD";
      case State::STRONG_BACKOFF: return "STRONG_BACKOFF";
      case State::INSERT_FORK: return "INSERT_FORK";
      case State::DETACH: return "DETACH";
      case State::BACKOFF: return "BACKOFF";
      case State::VERIFY_RELEASE: return "VERIFY_RELEASE";
      case State::AREA_VERIFY: return "AREA_VERIFY";
      case State::ASCEND: return "ASCEND";
      case State::COMPLETE: return "COMPLETE";
    }
    return "UNKNOWN";
  }

  // MAVROS OverrideRCIn: CHAN_NOCHANGE로 채운 뒤 제어할 채널만 덮어쓴다.
  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral_control(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
  }

  // RC override를 끊고 해당 채널을 수동/다른 소스에 반환.
  void release_controlled_channels(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
    rc_monitor_pub_->publish(msg);
  }

  // --- 파라미터 캐시 ---
  std::string bbox_topic_;
  std::string depth_pose_topic_;
  double depth_pose_scale_{-1.0};
  double depth_pose_offset_m_{0.0};
  std::string state_topic_;
  std::string rc_override_topic_;
  std::string rc_monitor_topic_;
  double control_rate_hz_{20.0};
  double detection_timeout_sec_{1.0};
  double surface_depth_m_{0.4};
  double target_depth_m_{1.0};
  double depth_tolerance_m_{0.2};
  double depth_kp_{0.3};
  double depth_ki_{0.05};
  double depth_kd_{0.04};
  double depth_bias_{0.1};
  double depth_integral_limit_{2.0};
  double heave_limit_{0.4};
  int max_depth_delta_pwm_{160};
  double lpf_tau_sec_{0.3};
  int buoy_class_id_{0};
  int stick_class_id_{1};
  int min_detection_hits_{3};
  int target_confirm_hits_{3};
  double target_confirm_sec_{0.2};
  double approach_area_ratio_{0.20};
  double search_timeout_sec_{40.0};
  double area_verify_sec_{12.0};
  double buoy_area_similar_ratio_{0.15};
  double buoy_confidence_similar_delta_{0.05};
  double buoy_same_target_center_ratio_{0.12};
  double fork_target_x_{0.30};
  double fork_target_y_{0.70};
  double align_zone_min_x_{0.60};
  double align_zone_max_y_{0.40};
  double align_stable_sec_{0.7};
  int insert_pwm_{1560};
  double insert_duration_sec_{0.8};
  int detach_pwm_{1620};
  double detach_duration_sec_{0.3};
  int backoff_pwm_{1420};
  double backoff_duration_sec_{0.5};
  int strong_forward_pwm_{1700};
  double strong_forward_duration_sec_{0.8};
  int strong_backoff_pwm_{1300};
  double strong_backoff_duration_sec_{0.8};
  double verify_clear_sec_{1.0};
  double verify_timeout_sec_{3.0};
  int max_target_retries_{2};
  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  int max_yaw_delta_{180};
  int max_tracking_depth_delta_{100};
  int approach_forward_pwm_{1700};
  int approach_forward_min_pwm_{1560};
  double approach_vision_throttle_weight_{0.4};
  int search_yaw_pwm_{1530};
  int search_forward_pwm_{1520};
  int reacquire_yaw_pwm_{1470};
  double reacquire_yaw_duration_sec_{0.5};
  double reacquire_timeout_sec_{1.0};
  bool yaw_invert_{false};
  bool vertical_positive_is_up_{true};

  // --- 런타임 상태 ---
  State state_{State::DIVE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  // ALIGN deadband 유지 등 "조건 지속 시간" 측정용
  std::optional<rclcpp::Time> condition_started_at_;
  std::optional<rclcpp::Time> target_confirm_started_at_;
  std::optional<double> depth_m_;
  rclcpp::Time depth_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<double> mission_hold_depth_m_;
  bool depth_pid_initialized_{false};
  rclcpp::Time last_depth_control_time_{0, 0, RCL_ROS_TIME};
  double previous_depth_error_m_{0.0};
  double depth_error_integral_{0.0};
  std::optional<Detection> buoy_;
  std::optional<Detection> stick_;
  int target_retries_{0};

  // --- ROS 인터페이스 ---
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionStateMachineNode>();
  rclcpp::spin(node);
  node->publish_release_once();  // 종료 시 RC override 해제
  rclcpp::shutdown();
  return 0;
}
