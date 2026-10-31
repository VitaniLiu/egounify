"""Full recorded joint-path collision check with conservative tool envelopes.

Runs on the robot host, reads ROS model configuration, and sends no commands.
"""
import argparse
import json
import math
from pathlib import Path

import coal  # Load the ROS Jazzy collision ABI before pinocchio.
import hppfcl
import numpy as np
import pinocchio as pin
import rclpy
from rcl_interfaces.srv import GetParameters


def robot_description():
    rclpy.init(domain_id=29)
    node = rclpy.create_node("full_path_collision_audit")
    client = node.create_client(GetParameters, "/tj/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError("robot_description parameter service unavailable")
    request = GetParameters.Request(); request.names = ["robot_description"]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or not future.result():
        raise RuntimeError("robot_description unavailable")
    xml = future.result().values[0].string_value
    node.destroy_node(); rclpy.shutdown()
    return xml


def main(path, output):
    source = json.loads(path.read_text())
    xml = robot_description()
    model = pin.buildModelFromXML(xml)
    package_dirs = ["/opt/kernelmind/apex/install/marvin_description/share"]
    geometry = pin.buildGeomFromUrdfString(model, xml, pin.GeometryType.COLLISION, package_dirs)

    # Model the installed gripper, wrist camera, bracket and local cable bundle as
    # a conservative 16 cm radius envelope centred 8 cm beyond each flange.
    envelope = {"radius_m": 0.16, "flange_offset_m": [0.0, 0.0, 0.08]}
    for side in ("L", "R"):
        frame = model.getFrameId("flange_" + side)
        joint = model.frames[frame].parentJoint
        placement = model.frames[frame].placement * pin.SE3(np.eye(3), np.array(envelope["flange_offset_m"]))
        obj = pin.GeometryObject("tool_envelope_" + side, joint, frame, placement, coal.Sphere(envelope["radius_m"]))
        geometry.addGeometryObject(obj)

    geometry.removeAllCollisionPairs(); geometry.addAllCollisionPairs()
    # Adjacent bodies are expected to touch at their joint. Keep all other
    # robot-robot and both tool-envelope pairs.
    kept = []
    def kinematic_distance(a, b):
        seen = {a: 0}; frontier = [a]
        while frontier:
            joint = frontier.pop(0); distance = seen[joint]
            if joint == b: return distance
            neighbours = [model.parents[joint]] + [i for i, parent in enumerate(model.parents) if parent == joint]
            for neighbour in neighbours:
                if neighbour not in seen: seen[neighbour] = distance + 1; frontier.append(neighbour)
        return 999
    for pair in geometry.collisionPairs:
        a, b = geometry.geometryObjects[pair.first], geometry.geometryObjects[pair.second]
        ja, jb = a.parentJoint, b.parentJoint
        # Meshes up to two joints apart overlap or run within millimetres by
        # construction; this also excludes each tool envelope from its own wrist.
        adjacent = kinematic_distance(ja, jb) <= 2
        if not adjacent:
            kept.append(pin.CollisionPair(pair.first, pair.second))
    geometry.removeAllCollisionPairs()
    for pair in kept: geometry.addCollisionPair(pair)
    data = model.createData(); geom_data = pin.GeometryData(geometry)
    index = {name: i for i, name in enumerate(model.names)}
    collisions = []
    path_samples = source["samples"]
    # The LeRobot execution contract has 314 frames. Select 314 uniformly
    # distributed controller-output states, including both endpoints.
    selected = np.rint(np.linspace(0, len(path_samples) - 1, 314)).astype(int)
    for frame_index, source_index in enumerate(selected):
        sample_index = int(source_index); sample = path_samples[sample_index]
        q = pin.neutral(model)
        for name, value in sample["position"].items():
            if name not in index: raise RuntimeError("Joint missing from URDF: " + name)
            q[model.joints[index[name]].idx_q] = value
        pin.forwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geometry, geom_data, q)
        for pair_index, pair in enumerate(geometry.collisionPairs):
            a, b = geometry.geometryObjects[pair.first], geometry.geometryObjects[pair.second]
            request = coal.CollisionRequest()
            result = coal.CollisionResult()
            if coal.collide(a.geometry, geom_data.oMg[pair.first], b.geometry, geom_data.oMg[pair.second], request, result):
                collisions.append({"frame": frame_index, "source_sample": sample_index, "stamp_ns": sample["stamp_ns"], "a": a.name, "b": b.name})
                if len(collisions) >= 100: break
        if len(collisions) >= 100: break
    report = {"frames_checked": frame_index + 1, "source_controller_samples": len(path_samples), "collision_pairs_checked": len(geometry.collisionPairs), "tool_envelope": envelope,
              "method": "exact mesh/envelope intersection at all 314 replay frames; kinematic neighbours within two joints excluded",
              "collisions": collisions, "passed": not collisions}
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(); parser.add_argument("path", type=Path); parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); raise SystemExit(main(args.path, args.output))
