from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bbox_topic", default_value="/vision/buoy_bbox"),
            DeclareLaunchArgument("depth_pose_topic", default_value="/depth/pose"),
            DeclareLaunchArgument("depth_pose_scale", default_value="-1.0"),
            DeclareLaunchArgument("depth_pose_offset_m", default_value="0.0"),
            DeclareLaunchArgument("state_topic", default_value="/mission/state"),
            DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
            DeclareLaunchArgument("rc_monitor_topic", default_value="/mission/rc_command"),
            DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("detection_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("throttle_channel", default_value="3"),
            DeclareLaunchArgument("yaw_channel", default_value="4"),
            DeclareLaunchArgument("forward_channel", default_value="5"),
            DeclareLaunchArgument("neutral_pwm", default_value="1500"),
            DeclareLaunchArgument(
                "min_pwm",
                default_value="1300",
                description="Minimum thruster PWM command.",
            ),
            DeclareLaunchArgument(
                "max_pwm",
                default_value="1700",
                description="Maximum thruster PWM command.",
            ),
            # 아래 기본값은 mission_state_machine_node.cpp declare_parameter 와 동기화
            DeclareLaunchArgument(
                "yaw_kp_pwm",
                default_value="100.0",
                description="Yaw proportional gain in PWM per normalized horizontal error.",
            ),
            DeclareLaunchArgument(
                "max_yaw_delta",
                default_value="100",
                description="Absolute safety limit for yaw PWM deviation from neutral.",
            ),
            DeclareLaunchArgument(
                "forward_pwm",
                default_value="1700",
                description="Maps to approach_forward_pwm (max approach forward).",
            ),
            DeclareLaunchArgument("approach_forward_min_pwm", default_value="1560"),
            DeclareLaunchArgument("search_yaw_pwm", default_value="1560"),
            DeclareLaunchArgument("search_forward_pwm", default_value="1520"),
            DeclareLaunchArgument("reacquire_yaw_pwm", default_value="1470"),
            DeclareLaunchArgument("reacquire_yaw_duration_sec", default_value="0.5"),
            DeclareLaunchArgument("reacquire_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("yaw_invert", default_value="false"),
            DeclareLaunchArgument("vertical_positive_is_up", default_value="true"),
            DeclareLaunchArgument("surface_depth_m", default_value="0.0"),
            DeclareLaunchArgument(
                "target_depth_m",
                default_value="0.05",
                description="Depth reached immediately after node startup (positive-down, m).",
            ),
            DeclareLaunchArgument("depth_kp", default_value="0.3"),
            DeclareLaunchArgument("depth_ki", default_value="0.05"),
            DeclareLaunchArgument("depth_kd", default_value="0.04"),
            DeclareLaunchArgument("depth_bias", default_value="0.1"),
            DeclareLaunchArgument("depth_integral_limit", default_value="2.0"),
            DeclareLaunchArgument("heave_limit", default_value="0.4"),
            DeclareLaunchArgument("lpf_tau_sec", default_value="0.3"),
            DeclareLaunchArgument("buoy_class_id", default_value="0"),
            DeclareLaunchArgument("stick_class_id", default_value="1"),
            DeclareLaunchArgument("min_detection_hits", default_value="3"),
            DeclareLaunchArgument("target_confirm_hits", default_value="3"),
            DeclareLaunchArgument("target_confirm_sec", default_value="0.2"),
            DeclareLaunchArgument("approach_area_ratio", default_value="0.10"),
            DeclareLaunchArgument("approach_vision_throttle_weight", default_value="0.4"),
            DeclareLaunchArgument("fork_target_x", default_value="0.30"),
            DeclareLaunchArgument("fork_target_y", default_value="0.70"),
            DeclareLaunchArgument(
                "buoy_align_target_x",
                default_value="0.85",
                description="Buoy center control target for ALIGN_STICK; 1.0 is the far right.",
            ),
            DeclareLaunchArgument(
                "buoy_align_target_y",
                default_value="0.25",
                description="Buoy center control target for ALIGN_STICK; 0.0 is the top.",
            ),
            DeclareLaunchArgument(
                "align_zone_min_x",
                default_value="0.75",
                description="ALIGN_STICK success zone: buoy center must be at or right of this x.",
            ),
            DeclareLaunchArgument(
                "align_zone_max_y",
                default_value="0.35",
                description="ALIGN_STICK success zone: buoy center must be at or above this y.",
            ),
            DeclareLaunchArgument("align_stable_sec", default_value="0.7"),
            DeclareLaunchArgument(
                "align_timeout_sec",
                default_value="10.0",
                description="Force STRONG_FORWARD when ALIGN_STICK exceeds this time.",
            ),
            DeclareLaunchArgument("insert_pwm", default_value="1560"),
            DeclareLaunchArgument("insert_duration_sec", default_value="0.8"),
            DeclareLaunchArgument("detach_pwm", default_value="1620"),
            DeclareLaunchArgument("detach_duration_sec", default_value="0.3"),
            DeclareLaunchArgument("backoff_pwm", default_value="1420"),
            DeclareLaunchArgument("backoff_duration_sec", default_value="0.5"),
            DeclareLaunchArgument("strong_forward_pwm", default_value="1700"),
            DeclareLaunchArgument("strong_forward_duration_sec", default_value="1.2"),
            DeclareLaunchArgument("strong_backoff_pwm", default_value="1300"),
            DeclareLaunchArgument("strong_backoff_duration_sec", default_value="1.2"),
            DeclareLaunchArgument("search_timeout_sec", default_value="40.0"),
            DeclareLaunchArgument("area_verify_sec", default_value="12.0"),
            Node(
                package="auv_test_vision",
                executable="mission_state_machine_node",
                name="mission_state_machine_node",
                output="screen",
                parameters=[
                    {
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                        "depth_pose_scale": ParameterValue(
                            LaunchConfiguration("depth_pose_scale"), value_type=float
                        ),
                        "depth_pose_offset_m": ParameterValue(
                            LaunchConfiguration("depth_pose_offset_m"), value_type=float
                        ),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                        "rc_monitor_topic": LaunchConfiguration("rc_monitor_topic"),
                        "control_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
                        "detection_timeout_sec": ParameterValue(
                            LaunchConfiguration("detection_timeout_sec"), value_type=float
                        ),
                        "surface_depth_m": ParameterValue(
                            LaunchConfiguration("surface_depth_m"), value_type=float
                        ),
                        "target_depth_m": ParameterValue(
                            LaunchConfiguration("target_depth_m"), value_type=float
                        ),
                        "buoy_class_id": ParameterValue(
                            LaunchConfiguration("buoy_class_id"), value_type=int
                        ),
                        "stick_class_id": ParameterValue(
                            LaunchConfiguration("stick_class_id"), value_type=int
                        ),
                        "approach_area_ratio": ParameterValue(
                            LaunchConfiguration("approach_area_ratio"), value_type=float
                        ),
                        "approach_vision_throttle_weight": ParameterValue(
                            LaunchConfiguration("approach_vision_throttle_weight"),
                            value_type=float,
                        ),
                        "min_detection_hits": ParameterValue(
                            LaunchConfiguration("min_detection_hits"), value_type=int
                        ),
                        "target_confirm_hits": ParameterValue(
                            LaunchConfiguration("target_confirm_hits"), value_type=int
                        ),
                        "target_confirm_sec": ParameterValue(
                            LaunchConfiguration("target_confirm_sec"), value_type=float
                        ),
                        "fork_target_x": ParameterValue(
                            LaunchConfiguration("fork_target_x"), value_type=float
                        ),
                        "fork_target_y": ParameterValue(
                            LaunchConfiguration("fork_target_y"), value_type=float
                        ),
                        "buoy_align_target_x": ParameterValue(
                            LaunchConfiguration("buoy_align_target_x"), value_type=float
                        ),
                        "buoy_align_target_y": ParameterValue(
                            LaunchConfiguration("buoy_align_target_y"), value_type=float
                        ),
                        "align_zone_min_x": ParameterValue(
                            LaunchConfiguration("align_zone_min_x"), value_type=float
                        ),
                        "align_zone_max_y": ParameterValue(
                            LaunchConfiguration("align_zone_max_y"), value_type=float
                        ),
                        "align_stable_sec": ParameterValue(
                            LaunchConfiguration("align_stable_sec"), value_type=float
                        ),
                        "align_timeout_sec": ParameterValue(
                            LaunchConfiguration("align_timeout_sec"), value_type=float
                        ),
                        "insert_pwm": ParameterValue(
                            LaunchConfiguration("insert_pwm"), value_type=int
                        ),
                        "insert_duration_sec": ParameterValue(
                            LaunchConfiguration("insert_duration_sec"), value_type=float
                        ),
                        "detach_pwm": ParameterValue(
                            LaunchConfiguration("detach_pwm"), value_type=int
                        ),
                        "detach_duration_sec": ParameterValue(
                            LaunchConfiguration("detach_duration_sec"), value_type=float
                        ),
                        "backoff_pwm": ParameterValue(
                            LaunchConfiguration("backoff_pwm"), value_type=int
                        ),
                        "backoff_duration_sec": ParameterValue(
                            LaunchConfiguration("backoff_duration_sec"), value_type=float
                        ),
                        "strong_forward_pwm": ParameterValue(
                            LaunchConfiguration("strong_forward_pwm"), value_type=int
                        ),
                        "strong_forward_duration_sec": ParameterValue(
                            LaunchConfiguration("strong_forward_duration_sec"), value_type=float
                        ),
                        "strong_backoff_pwm": ParameterValue(
                            LaunchConfiguration("strong_backoff_pwm"), value_type=int
                        ),
                        "strong_backoff_duration_sec": ParameterValue(
                            LaunchConfiguration("strong_backoff_duration_sec"), value_type=float
                        ),
                        "search_timeout_sec": ParameterValue(
                            LaunchConfiguration("search_timeout_sec"), value_type=float
                        ),
                        "area_verify_sec": ParameterValue(
                            LaunchConfiguration("area_verify_sec"), value_type=float
                        ),
                        "depth_kp": ParameterValue(
                            LaunchConfiguration("depth_kp"), value_type=float
                        ),
                        "depth_ki": ParameterValue(
                            LaunchConfiguration("depth_ki"), value_type=float
                        ),
                        "depth_kd": ParameterValue(
                            LaunchConfiguration("depth_kd"), value_type=float
                        ),
                        "depth_bias": ParameterValue(
                            LaunchConfiguration("depth_bias"), value_type=float
                        ),
                        "depth_integral_limit": ParameterValue(
                            LaunchConfiguration("depth_integral_limit"), value_type=float
                        ),
                        "heave_limit": ParameterValue(
                            LaunchConfiguration("heave_limit"), value_type=float
                        ),
                        "lpf_tau_sec": ParameterValue(
                            LaunchConfiguration("lpf_tau_sec"), value_type=float
                        ),
                        "throttle_channel": ParameterValue(
                            LaunchConfiguration("throttle_channel"), value_type=int
                        ),
                        "yaw_channel": ParameterValue(
                            LaunchConfiguration("yaw_channel"), value_type=int
                        ),
                        "forward_channel": ParameterValue(
                            LaunchConfiguration("forward_channel"), value_type=int
                        ),
                        "neutral_pwm": ParameterValue(
                            LaunchConfiguration("neutral_pwm"), value_type=int
                        ),
                        "min_pwm": ParameterValue(
                            LaunchConfiguration("min_pwm"), value_type=int
                        ),
                        "max_pwm": ParameterValue(
                            LaunchConfiguration("max_pwm"), value_type=int
                        ),
                        "yaw_kp_pwm": ParameterValue(
                            LaunchConfiguration("yaw_kp_pwm"), value_type=float
                        ),
                        "max_yaw_delta": ParameterValue(
                            LaunchConfiguration("max_yaw_delta"), value_type=int
                        ),
                        "approach_forward_pwm": ParameterValue(
                            LaunchConfiguration("forward_pwm"), value_type=int
                        ),
                        "approach_forward_min_pwm": ParameterValue(
                            LaunchConfiguration("approach_forward_min_pwm"), value_type=int
                        ),
                        "search_yaw_pwm": ParameterValue(
                            LaunchConfiguration("search_yaw_pwm"), value_type=int
                        ),
                        "search_forward_pwm": ParameterValue(
                            LaunchConfiguration("search_forward_pwm"), value_type=int
                        ),
                        "reacquire_yaw_pwm": ParameterValue(
                            LaunchConfiguration("reacquire_yaw_pwm"), value_type=int
                        ),
                        "reacquire_yaw_duration_sec": ParameterValue(
                            LaunchConfiguration("reacquire_yaw_duration_sec"), value_type=float
                        ),
                        "reacquire_timeout_sec": ParameterValue(
                            LaunchConfiguration("reacquire_timeout_sec"), value_type=float
                        ),
                        "yaw_invert": ParameterValue(
                            LaunchConfiguration("yaw_invert"), value_type=bool
                        ),
                        "vertical_positive_is_up": ParameterValue(
                            LaunchConfiguration("vertical_positive_is_up"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
