"""Export and validate the controller-output joint path from one MCAP recording.

This tool never connects to ROS and never sends robot commands.
"""
import argparse
import json
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages


GROUPS = {
    "/tj/control/joint_cmd_A": [f"Joint{i}_L" for i in range(1, 8)],
    "/tj/control/joint_cmd_B": [f"Joint{i}_R" for i in range(1, 8)],
    "/tj/control/joint_cmd_body": ["waist_joint_1", "waist_joint_2", "waist_joint_3"],
    "/tj/control/joint_cmd_head": ["Head_J1_Joint", "Head_J2_Joint"],
}
LIMITS = {"waist_joint_1": 1.0}
LIMITS.update({name: 3.14 for names in GROUPS.values() for name in names if name != "waist_joint_1"})


def export(source):
    messages = []
    for item in read_ros2_messages(str(source), topics=list(GROUPS)):
        names = GROUPS[item.channel.topic]
        values = list(item.ros_msg.positions)[: len(names)]
        if len(values) != len(names):
            raise ValueError(f"Malformed {item.channel.topic}")
        header = item.ros_msg.header.stamp
        stamp = int(header.sec) * 1_000_000_000 + int(header.nanosec)
        if stamp <= 0:
            raise ValueError(f"Missing controller header timestamp on {item.channel.topic}")
        messages.append((stamp, item.channel.topic, names, values))
    messages.sort()
    latest = {}
    last = {}
    maxima = {name: 0.0 for name in LIMITS}
    violations = []
    samples = []
    pending_stamp = None
    for stamp, topic, names, values in messages:
        if pending_stamp is not None and stamp != pending_stamp and len(latest) == 19:
            samples.append({"stamp_ns": pending_stamp, "position": dict(latest)})
        pending_stamp = stamp
        if topic in last:
            prior_stamp, prior = last[topic]
            dt = (stamp - prior_stamp) / 1e9
            if dt <= 0:
                raise ValueError("Non-increasing controller timestamp")
            for name, a, b in zip(names, prior, values):
                velocity = abs(b - a) / dt
                maxima[name] = max(maxima[name], velocity)
                if velocity > LIMITS[name] + 1e-6:
                    violations.append({"joint": name, "velocity": velocity, "limit": LIMITS[name], "stamp_ns": stamp})
        last[topic] = (stamp, values)
        latest.update(zip(names, values))
    if pending_stamp is not None and len(latest) == 19:
        samples.append({"stamp_ns": pending_stamp, "position": dict(latest)})
    return {
        "source": str(source),
        "controller_messages": len(messages),
        "timestamp_source": "ROS message header",
        "samples": samples,
        "max_target_velocity_by_joint": maxima,
        "velocity_limits": LIMITS,
        "velocity_violations": violations,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = export(args.source)
    args.output.write_text(json.dumps(result) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    raise SystemExit(bool(result["velocity_violations"]))
