#!/usr/bin/env bash
# ExecStart entrypoint for apex-teleop.service: resolve the robot model from
# /etc/apex/apex.env, then launch teleop_replay (which itself re-derives
# base_model/arm_model for the pro byconfig launch chain).

source /etc/apex/apex_ros_env.sh

# Reliable replay command transport across local ROS processes.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

platform="${APEX_ROBOT_PLATFORM:-gento}"
case "${platform}" in
  gento)
    APEX_ROBOT_MODEL="${APEX_ROBOT_MODEL:-luna}"
    case "${APEX_ROBOT_MODEL}" in
      lunav3) APEX_ROBOT_MODEL="luna_v3" ;;
      skyev2|skye_v2|skye920|gento_skye_v2|gento_skye_v2_920) APEX_ROBOT_MODEL="skye_v2_920" ;;
      sky) APEX_ROBOT_MODEL="skye" ;;
    esac
    ;;
  pro)
    APEX_ROBOT_MODEL="${APEX_ROBOT_MODEL:-pro_m6s_696_new}"
    case "${APEX_ROBOT_MODEL}" in
      pro_*_*) ;;
      *)
        echo "Unsupported APEX_ROBOT_MODEL=${APEX_ROBOT_MODEL}; expected pro_arm_base, e.g. pro_m6s_696_new" >&2
        exit 2
        ;;
    esac
    ;;
  *)
    echo "Unsupported APEX_ROBOT_PLATFORM=${platform}; valid values: gento, pro" >&2
    exit 2
    ;;
esac

APEX_GLOVE_MODE="${APEX_GLOVE_MODE:-false}"
case "${APEX_TELEOP_MODE:-controller}" in
  dexhand) APEX_GLOVE_MODE=true ;;
esac
case "${APEX_TOOL_TYPE:-dm}" in
  wuji|wujihand) APEX_GLOVE_MODE=true ;;
esac

# Namespace comes from APEX_ROS_NAMESPACE in the environment (apex.env); do NOT
# pass ros_namespace:= — that key is reserved by launch_ros and setting it here
# double-applies the namespace (/tj/tj).
exec ros2 launch apex_backend teleop_replay.launch.py \
  robot_platform:="${platform}" \
  robot_model:="${APEX_ROBOT_MODEL}" \
  glove_mode:="${APEX_GLOVE_MODE}" \
  use_rviz:="${APEX_USE_RVIZ:-false}"
