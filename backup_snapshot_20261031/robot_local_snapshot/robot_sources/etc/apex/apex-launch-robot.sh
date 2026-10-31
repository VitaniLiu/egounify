#!/usr/bin/env bash
# ExecStart entrypoint for apex-robot.service. robot_service.launch.py keeps
# robot bringup and the five-minute rolling MCAP recorder in the same launch
# process so systemd starts and stops them together.

source /etc/apex/apex_ros_env.sh

platform="${APEX_ROBOT_PLATFORM:-gento}"
case "${platform}" in
  gento)
    robot_model="${APEX_ROBOT_MODEL:-luna}"
    case "${robot_model}" in
      lunav3) robot_model="luna_v3" ;;
      skyev2|skye_v2|skye920|gento_skye_v2|gento_skye_v2_920) robot_model="skye_v2_920" ;;
      sky) robot_model="skye" ;;
    esac
    case "${robot_model}" in
      luna|luna_v3|skye|skye_v2_920) ;;
      *)
        echo "Unsupported Gento APEX_ROBOT_MODEL=${robot_model}; expected luna, luna_v3, skye or skye_v2_920" >&2
        exit 2
        ;;
    esac
    ;;
  pro)
    robot_model="${APEX_ROBOT_MODEL:-pro_m6s_696_new}"
    case "${robot_model}" in
      pro_*_*) ;;
      *)
        echo "Unsupported Pro APEX_ROBOT_MODEL=${robot_model}; expected pro_arm_base, e.g. pro_m6s_696_new" >&2
        exit 2
        ;;
    esac
    ;;
  *)
    echo "Unsupported APEX_ROBOT_PLATFORM=${platform}; valid values: gento, pro" >&2
    exit 2
    ;;
esac

exec ros2 launch apex_backend robot_service.launch.py \
  robot_platform:="${platform}" \
  robot_model:="${robot_model}" \
  apex_ros_namespace:="${APEX_ROS_NAMESPACE:-tj}" \
  use_rviz:="${APEX_USE_RVIZ:-false}"
