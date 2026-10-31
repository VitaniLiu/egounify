#!/usr/bin/env bash
# Run on ms-thor as root. No recordings, gains, or robot safety flags are changed.
set -euo pipefail
[[ $(id -u) == 0 ]] || { echo 'Run with sudo on ms-thor'; exit 1; }
[[ $(hostname) == ms-thor ]] || { echo 'Wrong host'; exit 1; }
export ROS_LOG_DIR=/tmp/replay-chain-audit
set +u
source /etc/apex/apex_ros_env.sh
set -u
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Refuse a restart during motion, VR control, recording or native replay.
python3 - <<'PY'
import json,time,math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,String
rclpy.init(domain_id=29);n=rclpy.create_node('teleop_udp_repair_preflight');s={}
def joint(m):s.update(j=m,t=time.monotonic())
n.create_subscription(JointState,'/tj/joint_states',joint,qos_profile_sensor_data)
n.create_subscription(Bool,'/tj/info/vr_connected',lambda m:s.update(vr=m.data),qos_profile_sensor_data)
n.create_subscription(String,'/tj/info/gento_replay/status',lambda m:s.update(native=json.loads(m.data),nt=time.monotonic()),10)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert time.monotonic()-s.get('t',0)<.2,'No fresh joint feedback'
assert len(s['j'].velocity)==19 and all(math.isfinite(v) and abs(v)<.02 for v in s['j'].velocity),'Robot moving/invalid feedback'
assert s.get('vr') is False,'VR connected or unknown'
assert time.monotonic()-s.get('nt',0)<2,'No native replay status'
assert not s['native'].get('playback_active') and not s['native'].get('recording'),'Replay/recording active'
n.destroy_node();rclpy.shutdown()
PY
conf=/etc/systemd/system/apex-teleop.service.d/90-replay-udp.conf
install -d -m 755 "$(dirname "$conf")"
if [[ -e "$conf" ]]; then cp -a "$conf" "$conf.backup-$(date +%Y%m%d-%H%M%S)"; fi
printf '[Service]\nEnvironment=FASTDDS_BUILTIN_TRANSPORTS=UDPv4\n' > "$conf"
# Stop temporary diagnostics before starting the original system service.
robot_uid=$(id -u user)
for unit in apex-teleop-udp-audit.service apex-teleop-replay-audit.service; do
  if runuser -u user -- env XDG_RUNTIME_DIR="/run/user/$robot_uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$robot_uid/bus" systemctl --user is-active --quiet "$unit"; then
    runuser -u user -- env XDG_RUNTIME_DIR="/run/user/$robot_uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$robot_uid/bus" systemctl --user stop "$unit"
  fi
done
systemctl daemon-reload
systemctl restart apex-teleop.service
systemctl is-active apex-teleop.service
printf 'UDP configured. This does not certify joint tracking or replay success.\n'
