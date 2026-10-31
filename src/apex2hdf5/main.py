from pathlib import Path
import argparse, json, os, shutil, signal, fcntl, hashlib, re

ROOT = Path(__file__).resolve().parents[2]
import sys
sys.path.insert(0,str(ROOT / "src"))
import unified_capture as unified
# Runtime data always lives on the identified WD USB disk, never beside code.
STORAGE_MOUNT = Path(
    os.environ.get(
        "APEX_STORAGE_MOUNT",
        (
            "/mnt/apex-data"
            if os.path.ismount("/mnt/apex-data")
            else "/media/descfly/My Passport"
        ),
    )
)
DATA = STORAGE_MOUNT / "apex_n_sharpa" / "data"
EXPECTED_SERIAL = "WD-WX12A367D1A2"
_STORAGE_DEVICE = None


def ensure_storage():
    import subprocess

    global _STORAGE_DEVICE
    mount = STORAGE_MOUNT.resolve()
    if not STORAGE_MOUNT.is_absolute() or not os.path.ismount(mount):
        raise RuntimeError(f"WD硬盘未挂载: {STORAGE_MOUNT}；拒绝回退到系统盘")
    device = subprocess.check_output(
        ["findmnt", "-rn", "-M", str(mount), "-o", "SOURCE"], text=True
    ).strip()
    serials = subprocess.check_output(
        ["lsblk", "-sn", "-o", "SERIAL", device], text=True
    ).split()
    if EXPECTED_SERIAL not in serials:
        raise RuntimeError(f"挂载设备不是指定WD硬盘: {device}, serial={serials}")
    if DATA.resolve() != mount / "apex_n_sharpa" / "data":
        raise RuntimeError("外置数据目录不允许通过符号链接转向其他位置")
    _STORAGE_DEVICE = mount.stat().st_dev
    return mount


def safe(path):
    if _STORAGE_DEVICE is None:
        ensure_storage()
    if (
        not os.path.ismount(STORAGE_MOUNT)
        or STORAGE_MOUNT.stat().st_dev != _STORAGE_DEVICE
    ):
        raise RuntimeError("WD硬盘挂载已改变，停止读写")
    p = Path(path).resolve()
    if DATA.resolve() not in p.parents:
        raise ValueError(f"数据路径必须位于外置硬盘 {DATA}: {p}")
    return p


def sync_directory(path):
    fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save(path, value):
    path = safe(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)
    sync_directory(path.parent)


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def lock(name):
    ensure_storage()
    DATA.mkdir(parents=True, exist_ok=True)
    f = (DATA / ("." + name + ".lock")).open("a")
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return f


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


signal.signal(signal.SIGTERM, interrupted)
import io
import numpy as np


def timestamps(values):
    a = np.asarray(values)
    if a.ndim != 1 or not len(a) or a.dtype.kind not in "iu":
        raise ValueError("时间戳必须为非空一维整数纳秒数组，禁止浮点时间戳")
    if any(int(v) < 0 or int(v) > np.iinfo(np.int64).max for v in a):
        raise ValueError("时间戳超出int64范围")
    a = a.astype("i8")
    if any(int(y) <= int(x) for x, y in zip(a[:-1], a[1:])):
        raise ValueError("时间戳必须唯一且严格递增，不能排序掩盖时钟回退")
    return a


def map_clock(values, source, target, mapping=None):
    a = timestamps(values)
    if not source or not target:
        raise ValueError("必须明确源/目标时钟域")
    if source == target and mapping is None:
        return a
    if not mapping or mapping.get("target_domain") != target:
        raise ValueError(f"缺少{source}到{target}的时钟标定")
    from fractions import Fraction

    scale = Fraction(str(mapping["scale"]))
    if scale <= 0:
        raise ValueError("时钟比例必须为有限正数")
    origin = mapping["source_origin_ns"]
    target_origin = mapping["target_origin_ns"]
    if type(origin) is not int or type(target_origin) is not int:
        raise ValueError("时钟原点必须为整数纳秒")
    # Rational arithmetic avoids float epoch truncation and int64 subtraction overflow.
    converted = [target_origin + round((int(v) - origin) * scale) for v in a]
    return timestamps(converted)


import h5py
from tqdm import tqdm
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image
import av


def field(obj, path):
    for key in path.split("."):
        if key:
            obj = obj[key] if isinstance(obj, dict) else getattr(obj, key)
    return obj


def pose(value):
    p = value.position
    q = value.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def stamp(message, log_ns, clock):
    if clock == "log":
        return log_ns
    if clock != "header":
        raise ValueError("clock必须为log/header")
    h = message.header.stamp
    if int(h.sec) < 0 or not 0 <= int(h.nanosec) < 10**9:
        raise ValueError("非法ROS header时间戳")
    t = int(h.sec) * 10**9 + int(h.nanosec)
    if t <= 0:
        raise ValueError("无效header时间戳")
    return t


def crop_image(image, crop):
    x0, y0, x1, y1 = crop
    if not (0 <= x0 < x1 <= image.width and 0 <= y0 < y1 <= image.height):
        raise ValueError("相机裁剪区域越界")
    return image.crop((x0, y0, x1, y1))


def encode(image):
    buf = io.BytesIO()
    image.convert("RGB").save(buf, format="JPEG", quality=95)
    return buf.getvalue()


def messages(payload, topics):
    files = sorted(payload.rglob("*.mcap"))
    if files:
        from contextlib import ExitStack
        import heapq

        def decoded(reader):
            for schema, channel, message, value in reader.iter_decoded_messages(
                topics=topics
            ):
                yield channel.topic, message.log_time, value

        # Split filenames are not a chronology (_10 may sort before _2).
        with ExitStack() as stack:
            readers = [
                make_reader(
                    stack.enter_context(safe(path).open("rb")),
                    decoder_factories=[DecoderFactory()],
                )
                for path in files
            ]
            yield from heapq.merge(
                *(decoded(reader) for reader in readers), key=lambda row: row[1]
            )
    else:
        # Explicit normalized adapter for recorders without MCAP. Bytes are base64.
        import base64

        source = safe(payload / "streams.jsonl")
        if not source.exists():
            raise ValueError("未找到MCAP或streams.jsonl")
        with source.open() as f:
            for line in f:
                item = json.loads(line)
                if item["topic"] in topics:
                    if "jpeg_base64" in item:
                        item["value"] = base64.b64decode(item["jpeg_base64"])
                    yield item["topic"], item["timestamp_ns"], item


class ModelPose:
    """FK from a pinned, self-contained MJCF and named measured joints."""

    def __init__(self, spec):
        import xml.etree.ElementTree as ET

        self.path = safe(spec["model"])
        blob = self.path.read_bytes()
        if hashlib.sha256(blob).hexdigest() != spec["sha256"]:
            raise ValueError("FK模型校验失败")
        root = ET.fromstring(blob)
        compiler = root.find("compiler")
        if root.findall(".//include") or (compiler is not None and
                compiler.get("angle", "degree") != "radian"):
            raise ValueError("FK要求独立、弧度制MJCF")
        nodes, parents = {}, {}

        def collect(body, parent):
            name = body.get("name")
            if not name or name in nodes:
                raise ValueError("FK body名称缺失或重复")
            nodes[name], parents[name] = body, parent
            for child in body.findall("body"):
                collect(child, name)

        for body in root.find("worldbody").findall("body"):
            collect(body, None)
        self.chains = {}
        for name in (spec["root"], spec["target"]):
            chain, current = [], name
            while current is not None:
                chain.append(nodes[current])
                current = parents[current]
            self.chains[name] = list(reversed(chain))
        self.root, self.target = spec["root"], spec["target"]

    @staticmethod
    def rotation(q):
        q = np.asarray(q, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
            raise ValueError("FK模型四元数不合法")
        w, x, y, z = q / np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    def __call__(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            raise ValueError("FK关节名称/位置不匹配")
        joints = dict(zip(msg.name, msg.position))
        if not np.isfinite(list(joints.values())).all():
            raise ValueError("FK关节反馈包含非有限值")

        def vector(node, key, default):
            return np.fromstring(node.get(key, default), sep=" ")

        def evaluate(chain):
            total = np.eye(4)
            for body in chain:
                if any(body.get(k) for k in ("euler", "axisangle", "xyaxes", "zaxis")):
                    raise ValueError("FK模型旋转仅支持显式quat")
                origin = np.eye(4)
                origin[:3, 3] = vector(body, "pos", "0 0 0")
                origin[:3, :3] = self.rotation(vector(body, "quat", "1 0 0 0"))
                total = total @ origin
                for joint in body.findall("joint"):
                    name = joint.get("name")
                    if name not in joints:
                        raise ValueError(f"FK缺少实测关节: {name}")
                    value = joints[name]
                    axis = vector(joint, "axis", "0 0 1")
                    if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
                        raise ValueError("FK关节轴不合法")
                    axis /= np.linalg.norm(axis)
                    motion = np.eye(4)
                    kind = joint.get("type", "hinge")
                    if kind == "slide":
                        motion[:3, 3] = axis * value
                    elif kind == "hinge":
                        x, y, z = axis
                        k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                        motion[:3, :3] = np.eye(3) + np.sin(value)*k + (1-np.cos(value))*(k@k)
                        pivot = vector(joint, "pos", "0 0 0")
                        motion[:3, 3] = pivot - motion[:3, :3] @ pivot
                    else:
                        raise ValueError(f"FK不支持关节类型: {kind}")
                    total = total @ motion
            return total

        t = np.linalg.inv(evaluate(self.chains[self.root])) @ evaluate(self.chains[self.target])
        return np.r_[t[:3, 3], matrix_quaternion(t[:3, :3])]


def matrix_quaternion(r):
    """Rotation matrix to xyzw, stable at pi rotations."""
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    return q


def poses_in_base(values, frame, transforms):
    """Convert XYZ+xyzw poses; source frame metadata remains independently checked."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 2 or values.shape[1] != 7 or not np.isfinite(values).all():
        raise ValueError("Invalid XYZ+xyzw poses")
    matrix = np.eye(4) if frame == "base_link" else transforms.get(frame)
    if matrix is None:
        raise ValueError(f"Missing {frame} -> base_link calibration")
    matrix = np.asarray(matrix, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-8, rtol=0)):
        raise ValueError("Invalid base_from_source SE(3)")
    r = matrix[:3, :3]
    rotation = matrix_quaternion(r)
    a, w = rotation[:3], rotation[3]
    q = values[:, 3:]
    norms = np.linalg.norm(q, axis=1)
    if np.any(abs(norms - 1) > .05) or np.any(norms < 1e-8):
        raise ValueError("Invalid source quaternion")
    q = q / norms[:, None]
    result = values.copy()
    result[:, :3] = values[:, :3] @ r.T + matrix[:3, 3]
    result[:, 3:6] = w*q[:, :3] + q[:, 3:]*a + np.cross(a, q[:, :3])
    result[:, 6] = w*q[:, 3] - q[:, :3] @ a
    return result


def convert(episode, output, config):
    payload = safe(episode / "payload")
    unified_mode=config.get('profile')==unified.PROFILE
    capture=unified.verify_raw(payload,config.get('allow_diagnostic',False),episode.name) if unified_mode else None
    mappings = config["streams"]
    collected = {k: [] for k in mappings}
    decoders = {}
    source_times = {k: {} for k in mappings}
    previous = {}
    fk = {name: ModelPose(spec["fk"]) for name, spec in mappings.items() if "fk" in spec}
    for name, spec in mappings.items():
        if name in fk and (spec["kind"] != "pose" or spec.get("source_role") != "state"
                           or spec.get("frame") != spec["fk"]["root"]):
            raise ValueError("FK仅用于明确根坐标系的实测state位姿")
        if name.startswith(("observations/", "actions/")):
            expected = "state" if name.startswith("observations/") else "action"
            if spec.get("source_role") != expected:
                raise ValueError(f"{name}: 必须显式声明source_role={expected}")
        if spec["kind"] == "pose" and spec.get("unit") != "m":
            raise ValueError(f"{name}: 位姿位置单位必须明确为m")
        if name.startswith("actions/"):
            peers = [
                value
                for key, value in mappings.items()
                if key.startswith("observations/")
            ]
            identity = lambda v: (
                v["topic"],
                v.get("field", ""),
                tuple(v.get("fields", [])),
            )
            if any(identity(peer) == identity(spec) for peer in peers):
                raise ValueError("禁止将同一源字段同时用于state/action")
    metadata = payload / "episode_metadata.json"
    capture_id = (
        json.loads(safe(metadata).read_text()).get("capture_id", "")
        if metadata.exists()
        else ""
    )
    if unified_mode:capture_id=capture['capture_id']
    by_topic = {}
    for name, spec in mappings.items():
        by_topic.setdefault(spec["topic"], []).append((name, spec))
    with h5py.File(safe(output), "w") as h:
        h.attrs.update(
            capture_id=capture_id,
            complete=False,
            schema_version=4,
            sample_hz=config["fps"],
            clock_domain=config["clock_domain"],
            metadata_json=json.dumps(config),
        )
        # Preserve every byte, including unselected topics, schemas, sidecars and video.
        for i, path in enumerate(sorted(payload.rglob("*"))):
            safe(path)
            if path.is_file():
                d = h.create_dataset(
                    f"raw_files/{i:06d}",
                    shape=(path.stat().st_size,),
                    dtype="u1",
                    chunks=True,
                )
                d.attrs["relative_path"] = str(path.relative_to(payload))
                with path.open("rb") as f:
                    offset = 0
                    for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
                        d[offset : offset + len(block)] = np.frombuffer(
                            block, dtype="u1"
                        )
                        offset += len(block)
        for topic, log_ns, msg in tqdm(
            messages(payload, list(by_topic)), desc=episode.name, unit="msg"
        ):
            for name, spec in by_topic[topic]:
                normalized = isinstance(msg, dict)
                if spec.get("complete_ik_request"):
                    # This export preserves the native nameless 19-position seed.
                    # Reject schema drift instead of silently dropping new fields.
                    if normalized:
                        raise ValueError("Complete IK extraction requires original ROS messages")
                    joint = msg.joint_state
                    if (len(msg.available) != 11 or len(joint.position) != 19
                            or joint.name or joint.velocity or joint.effort
                            or joint.header.frame_id or joint.header.stamp.sec or joint.header.stamp.nanosec):
                        raise ValueError("Unsupported recorded IK joint-state schema")
                raw_t = (
                    log_ns
                    if normalized
                    else stamp(msg, log_ns, spec.get("clock", "log"))
                )
                domain = (
                    msg.get("clock_domain")
                    if normalized
                    else (
                        config["clock_domain"]
                        if spec.get("clock", "log") == "log"
                        else spec.get("source_clock_domain")
                    )
                )
                t = int(
                    map_clock(
                        [raw_t], domain, config["clock_domain"], spec.get("clock_map")
                    )[0]
                )
                if name in previous and t <= previous[name]:
                    raise ValueError(f"{name}: 时间戳重复或回退")
                previous[name] = t
                source_times[name][t] = (raw_t, log_ns)
                if spec["kind"] == "pose" and name not in fk:
                    actual = (
                        msg.get("coordinate_frame")
                        if normalized
                        else getattr(getattr(msg, "header", None), "frame_id", None)
                    )
                    if not actual or actual != spec.get("frame"):
                        raise ValueError(f"{name}: 消息坐标系{actual!r}与配置不符")
                if (
                    normalized
                    and name.startswith(("observations/", "actions/"))
                    and msg.get("source_role") != spec["source_role"]
                ):
                    raise ValueError(f"{name}: 适配流source_role与配置不符")
                if name in fk:
                    if normalized:
                        raise ValueError("FK需要原始JointState消息")
                    value = fk[name](msg)
                else:
                    value = msg["value"] if normalized else field(msg, spec.get("field", ""))
                kind = spec["kind"]
                if kind == "vector" and "joint_names" in spec:
                    if normalized:
                        raise ValueError("命名关节提取需要原始JointState")
                    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                        raise ValueError("JointState关节名称/位置不匹配")
                    measured = dict(zip(msg.name, msg.position))
                    if not set(spec["joint_names"]) <= measured.keys():
                        raise ValueError("JointState缺少配置中的关节")
                    value = [measured[n] for n in spec["joint_names"]]
                if kind == "pose" and not normalized and name not in fk:
                    value = pose(value)
                elif (
                    kind == "vector"
                    and not normalized
                    and isinstance(spec.get("fields"), list)
                ):
                    value = [field(msg, x) for x in spec["fields"]]
                if kind == "vector" and "indices" in spec:
                    value = [value[i] for i in spec["indices"]]
                if kind == "image":
                    if normalized:
                        frames = [(t, Image.open(io.BytesIO(value)).convert("RGB"))]
                    elif "h264" in msg.format.lower():
                        if name not in decoders:
                            decoders[name] = av.CodecContext.create("h264", "r")
                        codec = decoders[name]
                        from fractions import Fraction

                        packet = av.Packet(bytes(value))
                        packet.pts = t
                        packet.dts = t
                        packet.time_base = Fraction(1, 10**9)
                        frames = [
                            (frame.pts, frame.to_image())
                            for frame in codec.decode(packet)
                        ]
                    else:
                        frames = [
                            (t, Image.open(io.BytesIO(bytes(value))).convert("RGB"))
                        ]
                    for ft, im in frames:
                        if ft is None:
                            raise ValueError("视频解码丢失PTS，拒绝按帧序号猜测")
                        if spec.get("crop"):
                            im = crop_image(im, spec["crop"])
                        if spec.get("size"):
                            im = im.resize(tuple(spec["size"]))
                        collected[name].append((ft, encode(im)))
                else:
                    collected[name].append((t, value))
        for name, codec in decoders.items():
            spec = mappings[name]
            for frame in codec.decode(None):
                if frame.pts is None:
                    raise ValueError("视频尾帧没有PTS")
                im = frame.to_image()
                if spec.get("crop"):
                    im = crop_image(im, spec["crop"])
                if spec.get("size"):
                    im = im.resize(tuple(spec["size"]))
                collected[name].append((frame.pts, encode(im)))
        bounds = []
        for name, spec in mappings.items():
            rows = collected[name]
            if not rows:
                if spec.get("required", True):
                    raise ValueError(f'必需数据流缺失: {name} ({spec["topic"]})')
                continue
            rows.sort(key=lambda r: r[0])
            ts = timestamps([r[0] for r in rows])
            if np.any(np.diff(ts) <= 0):
                raise ValueError(f"{name}: 重复时间戳，需检查分卷重叠/采集异常")
            group = h.require_group(name)
            group.create_dataset("timestamp_ns", data=ts)
            group.create_dataset(
                "source_timestamp_ns",
                data=[source_times[name][int(t)][0] for t in ts],
                dtype="i8",
            )
            group.create_dataset(
                "receive_timestamp_ns",
                data=[source_times[name][int(t)][1] for t in ts],
                dtype="i8",
            )
            group.attrs["source_role"] = spec.get("source_role", "image")
            for key in ("joint_names", "indices", "source_stage"):
                if key in spec:
                    group.attrs[key] = json.dumps(spec[key])
            if name in fk:
                group.attrs["derivation"] = "measured_joint_forward_kinematics"
                group.attrs["model_sha256"] = spec["fk"]["sha256"]
                group.attrs["target_frame"] = spec["fk"]["target"]
            group.attrs.update(
                clock_domain=config["clock_domain"],
                source_topic=spec["topic"],
                kind=spec["kind"],
                coordinate_frame=("base_link" if spec["kind"] == "pose" and name not in {"observations/chassis", "actions/chassis"} else spec.get("frame", "")),
                unit=spec.get("unit", ""),
            )
            if spec["kind"] == "image":
                d = group.create_dataset(
                    "jpeg", (len(rows),), dtype=h5py.vlen_dtype(np.dtype("uint8"))
                )
                for i, (_, v) in enumerate(rows):
                    d[i] = np.frombuffer(v, dtype="u1")
            else:
                values = np.asarray([r[1] for r in rows], dtype="f8")
                if values.ndim != 2:
                    raise ValueError(f"{name}: 值必须是二维数组")
                if spec["kind"] == "pose" and name not in {"observations/chassis", "actions/chassis"}:
                    values = poses_in_base(values, spec.get("frame"), config.get("frame_transforms", {}))
                    group.attrs["source_coordinate_frame"] = spec["frame"]
                group.create_dataset(
                    "eef" if spec["kind"] == "pose" else "value", data=values
                )
            bounds.append((ts[0], ts[-1]))
        if unified_mode:
            bounds.extend(unified.add_hands(h,payload,capture,config['hand_serials']))
            bounds.append((capture['start_ack_ns'],capture['stop_request_ns']))
            for role in ('observations','actions'):
                for side in ('left','right'):
                    if f'{role}/{side}_hand' not in h:raise ValueError('Missing unified hand stream')
        if not bounds:
            raise ValueError("无可转换的数据")
        start = int(max(x[0] for x in bounds))
        end = int(min(x[1] for x in bounds))
        if end <= start:
            raise ValueError("数据流无公共时间区间")
        period = round(1e9 / config["fps"])
        ts = start + np.arange((end - start) // period + 1, dtype="i8") * period
        h.create_dataset("time/sample_timestamp_ns", data=ts)
        for name, model in fk.items():
            if digest(model.path) != mappings[name]["fk"]["sha256"]:
                raise ValueError("运行期间FK模型改变")
            if unified_mode:
                d=h.create_dataset('provenance/models/'+name.replace('/','__'),data=np.frombuffer(model.path.read_bytes(),dtype='u1'))
                d.attrs.update(source_path=str(model.path),sha256=mappings[name]['fk']['sha256'])
        h.attrs.update(complete=True, frame_count=len(ts))
        h.flush()


def main():
    p = argparse.ArgumentParser(
        description="完整保留APEX原始数据，提取独立反馈和遥操流至HDF5"
    )
    p.add_argument("--task", required=True)
    p.add_argument("--episode", type=int)
    p.add_argument("--config", default=str(DATA / "config/apex.json"))
    a = p.parse_args()
    if not re.fullmatch(r"[\w-]+", a.task):
        p.error("task仅允许字母数字下划线及连字符")
    guard = lock("apex2hdf5")
    config_bytes = safe(a.config).read_bytes()
    config = json.loads(config_bytes)
    config_digest = hashlib.sha256(config_bytes).hexdigest()
    if not np.isfinite(config["fps"]) or not 0 < config["fps"] <= 1e9:
        raise ValueError("fps必须为正")
    source = safe(DATA / "raw_data/apex_data" / a.task)
    dest = safe(DATA / "hdf5" / a.task)
    dest.mkdir(parents=True, exist_ok=True)
    for path in dest.glob("*.partial"):
        path.unlink()
    episodes = sorted(source.glob("episode_[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]"))
    if a.episode is not None:episodes=[ep for ep in episodes if ep.name==f"episode_{a.episode:08d}"]
    if not episodes:
        raise ValueError("没有待转换的episode")
    for ep in tqdm(episodes, desc="apex2hdf5", unit="episode"):
        out = dest / ("hdf5_" + ep.name.split("_")[1] + ".hdf5")
        tmp = out.with_suffix(".partial")
        commit = out.with_suffix(".commit.json")
        checkpoint = dest / "progress.json"
        save(checkpoint, dict(episode=ep.name, status="running"))
        try:
            if digest(safe(a.config)) != config_digest:
                raise ValueError("运行期间配置改变，请重新启动命令")
            for spec in config["streams"].values():
                if "fk" in spec:
                    ModelPose(spec["fk"])
            manifest = json.loads(safe(ep / "manifest.json").read_text())
            payload = safe(ep / "payload")
            actual = {
                str(p.relative_to(payload)) for p in payload.rglob("*") if p.is_file()
            }
            if actual != {item["path"] for item in manifest["files"]}:
                raise ValueError("原始目录文件集合与manifest不符")
            for item in manifest["files"]:
                path = safe(payload / item["path"])
                if payload not in path.parents:
                    raise ValueError("manifest文件路径越过当前episode")
                if digest(path) != item["sha256"]:
                    raise ValueError(f"原始副本校验失败: {path}")
            if config.get('profile')==unified.PROFILE:
                certified=unified.verify_raw(payload,config.get('allow_diagnostic',False),ep.name)
                if manifest.get('episode_index')!=certified['episode_index'] or manifest.get('capture_id')!=certified['capture_id']:
                    raise ValueError('Import/capture episode identity mismatch')
            provenance = {
                "manifest": digest(safe(ep / "manifest.json")),
                "config": config_digest,
                "converter_revision": 5 if config.get("profile")==unified.PROFILE else 4,
            }
            if config.get('profile')==unified.PROFILE:
                provenance['implementation']={p.name:digest(p) for p in [Path(__file__),ROOT/'src/unified_capture.py']}
            if out.exists():
                if not commit.exists():
                    raise ValueError(f"输出缺少完整性记录，请移走旧输出后重建: {out}")
                record = json.loads(safe(commit).read_text())
                if record["provenance"] != provenance or record["sha256"] != digest(
                    safe(out)
                ):
                    raise ValueError(f"输出损坏或输入/配置已改变: {out}")
                save(checkpoint, dict(episode=ep.name, status="complete"))
                continue
            convert(safe(ep), tmp, config)
            # Recheck mutable inputs before publishing.
            if {
                str(p.relative_to(payload)) for p in payload.rglob("*") if p.is_file()
            } != actual:
                raise ValueError("转换期间原始文件集合改变")
            for item in manifest["files"]:
                if digest(safe(payload / item["path"])) != item["sha256"]:
                    raise ValueError("转换期间原始文件改变")
            if provenance["config"] != digest(safe(a.config)):
                raise ValueError("转换期间配置改变")
            with h5py.File(safe(tmp), "a") as h:
                h.attrs["provenance"] = json.dumps(provenance)
            with tmp.open("rb") as f:
                os.fsync(f.fileno())
            save(commit, {"provenance": provenance, "sha256": digest(tmp)})
            tmp.replace(out)
            sync_directory(dest)
            save(checkpoint, dict(episode=ep.name, status="complete"))
            tqdm.write(f"完成 {out.name}")
        except BaseException as exc:
            tmp.unlink(missing_ok=True)
            save(
                checkpoint, dict(episode=ep.name, status="interrupted", error=str(exc))
            )
            raise


if __name__ == "__main__":
    main()
